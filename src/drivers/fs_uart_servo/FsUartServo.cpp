/****************************************************************************
 *
 * FashionStar UART bus servo driver
 *
 ****************************************************************************/

#include "FsUartServo.hpp"

#include <errno.h>
#include <inttypes.h>
#include <stdint.h>
#include <string.h>
#include <sys/ioctl.h>

#include <lib/matrix/matrix/math.hpp>
#include <px4_platform_common/defines.h>

namespace
{
const char *default_uart_device_path()
{
#if defined(CONFIG_BOARD_SERIAL_TEL2)
	return CONFIG_BOARD_SERIAL_TEL2;
#elif defined(CONFIG_BOARD_SERIAL_TEL1)
	return CONFIG_BOARD_SERIAL_TEL1;
#else
	return "/dev/ttyS2";
#endif
}

/** FashionStar FSUS_PARAM_BAUDRATE 档位 + NuttX termios 可配置速率 */
static constexpr int k_supported_bauds[] = {9600, 19200, 38400, 57600, 115200, 250000, 500000, 1000000};

/* 平时控制窗口 5 ms（200 Hz）。FS_UT_FB=1 时每窗口最多 4 次查询（8 路各 100 Hz）。
 * 辨识扫频窗口由 IDEN_SV_HZ 决定，只查目标舵机，每窗口 1 次（与发送同频）。
 * 250 µs 服务节拍用来在两次发送之间收完应答。
 * 1 Mbaud 实测 RTT 中位约 0.8–1.0 ms；超时与「下帧同步前让路」必须同为 3 ms。
 * 按 100 Hz 辨识留空隙；IDEN_SV_HZ=200 时起步窗口只剩约 0.4 ms，查询容易被挤掉。 */
static constexpr hrt_abstime k_control_interval_us = 5_ms;
static constexpr uint32_t k_control_rate_hz = 200;
static constexpr uint32_t k_high_rate_feedback_hz_per_ch = 100;
static constexpr uint32_t k_legacy_feedback_hz_per_ch = 10;
static constexpr uint32_t k_high_rate_service_interval_us = 250;
static constexpr hrt_abstime k_high_rate_query_timeout_us = 3_ms;
static constexpr hrt_abstime k_legacy_query_timeout_us = 8_ms;
static constexpr hrt_abstime k_high_rate_post_control_quiet_us = 1_ms;
static constexpr hrt_abstime k_legacy_post_control_quiet_us = 2_ms;
static constexpr hrt_abstime k_query_control_guard_us = 3_ms;
static constexpr uint8_t k_max_query_budget = 4;

bool is_supported_baud(int baud)
{
	for (int b : k_supported_bauds) {
		if (baud == b) {
			return true;
		}
	}

	return false;
}

int normalize_baud(int baud)
{
	if (is_supported_baud(baud)) {
		return baud;
	}

	PX4_ERR("unsupported baud %i — using 115200 (FashionStar default)", baud);
	return 115200;
}

speed_t nuttx_baud_to_speed(int baud)
{
#ifndef B500000
#  define B500000 500000
#endif
#ifndef B1000000
#  define B1000000 1000000
#endif

	switch (baud) {
	case 9600:   return B9600;

	case 19200:  return B19200;

	case 38400:  return B38400;

	case 57600:  return B57600;

	case 115200: return B115200;

	case 250000: return 250000; /* NuttX: numeric → BOTHER custom rate */

	case 500000: return B500000;

	case 1000000: return B1000000;

	default:     return B115200;
	}
}

bool set_uart_baud(struct termios &uart_cfg, int baud)
{
	const speed_t speed = nuttx_baud_to_speed(baud);

	if (cfsetispeed(&uart_cfg, speed) != 0 || cfsetospeed(&uart_cfg, speed) != 0) {
		return false;
	}

	return true;
}

#if defined(__PX4_NUTTX)
/* NuttX O_RDWR = O_RDONLY|O_WRONLY (3). Do not OR host O_NOCTTY (may pollute flags). */
static constexpr int k_uart_open_flags = O_RDWR;
#else
static constexpr int k_uart_open_flags = O_RDWR | O_NOCTTY;
#endif

bool uart_verify_writeable(int fd)
{
	const int fl = fcntl(fd, F_GETFL, 0);

#if defined(__PX4_NUTTX)
	/* NuttX VFS returns -EACCES from write() when (f_oflags & O_WROK) == 0. */
	if ((fl & O_WRONLY) == 0) {
		PX4_ERR("uart not opened for write (fcntl=0%o, need O_WRONLY/O_RDWR)", fl);
		return false;
	}

#endif
	return true;
}
}

FsUartServo::FsUartServo(const char *device_path, int baud_cli) :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::serial_port_to_wq(device_path)),
	_baud_cli(baud_cli)
{
	memset(_device_path, 0, sizeof(_device_path));
	memset(_last_deg, 0, sizeof(_last_deg));

	strncpy(_device_path, device_path, sizeof(_device_path) - 1);
	_servo_iden_data_pub.advertise();
}

FsUartServo::~FsUartServo()
{
	close_uart();
}

void FsUartServo::refresh_servo_kill_state()
{
	bool kill = false;

	actuator_armed_s armed{};

	if (_actuator_armed_sub.copy(&armed)) {
		kill = armed.manual_lockdown || armed.lockdown || armed.force_failsafe;
	}

	manual_control_switches_s sw{};

	if (_manual_control_switches_sub.copy(&sw)) {
		kill = kill || (sw.kill_switch == manual_control_switches_s::SWITCH_POS_ON);
	}

	_servo_kill_zero = kill;
}

bool FsUartServo::init()
{
	update_params(true);
	refresh_servo_kill_state();

	/* 上电前已 kill：强制首帧按边沿逻辑发 0° */
	if (_servo_kill_zero) {
		_servo_kill_zero_prev = false;
		_sent_once = false;
		_last_sent_sample = 0;
	}

	/* UART is opened in Run() on the serial work queue (same task as write). */
	PX4_INFO("FashionStar servo: %s service=%" PRIu32 " Hz ctrl<=200 Hz iden=%" PRIu32 " Hz fb_target=%" PRIu32 " Hz/ch",
		 _device_path, 1000000U / _service_interval_us, iden_ctrl_hz(), feedback_target_hz_per_ch());

	return true;
}

void FsUartServo::close_uart()
{
	if (_uart_fd >= 0) {
#if defined(TIOCSSINGLEWIRE)
		if (_single_wire_active) {
			/* Do not leave the peripheral in HDSEL mode for the next owner. */
			(void)::ioctl(_uart_fd, TIOCSSINGLEWIRE, 0);
		}
#endif
		::close(_uart_fd);
		_uart_fd = -1;
	}

	_line_baud = -1;
	_single_wire_active = false;
}

bool FsUartServo::open_uart()
{
	close_uart();

	const int baud = normalize_baud((_baud_cli > 0) ? _baud_cli : _param_fs_ut_baud.get());

	_uart_fd = ::open(_device_path, k_uart_open_flags);

	if (_uart_fd < 0) {
		PX4_ERR("open %s failed (%i)", _device_path, errno);
		return false;
	}

	if (!uart_verify_writeable(_uart_fd)) {
		close_uart();
		return false;
	}

	_effective_baud = baud;

	/* Non-blocking I/O: required for ioctl(FIONREAD)+read() polling in Run(). */
	{
		const int fl = fcntl(_uart_fd, F_GETFL, 0);

		if (fl >= 0 && (fl & O_NONBLOCK) == 0) {
			fcntl(_uart_fd, F_SETFL, fl | O_NONBLOCK);
		}
	}

	struct termios uart_cfg {};
	tcgetattr(_uart_fd, &uart_cfg);

	uart_cfg.c_cflag &= ~PARENB; // clear parity
	uart_cfg.c_cflag &= ~CSTOPB; // 1 stop
	uart_cfg.c_cflag &= ~CSIZE;
	uart_cfg.c_cflag |= CS8;
	uart_cfg.c_cflag &= ~CRTSCTS;
	uart_cfg.c_cflag |= (CREAD | CLOCAL);

	uart_cfg.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

	uart_cfg.c_iflag &= ~(IXON | IXOFF | IXANY);
	uart_cfg.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);

	uart_cfg.c_oflag &= ~OPOST;

	if (!set_uart_baud(uart_cfg, baud)) {
		close_uart();
		PX4_ERR("cfsetispeed failed for baud %i", baud);
		return false;
	}

	if (tcsetattr(_uart_fd, TCSANOW, &uart_cfg) != 0) {
		close_uart();
		PX4_ERR("tcsetattr failed (%i)", errno);
		return false;
	}

	const bool single_wire = _param_fs_ut_1wire.get() != 0;

#if defined(TIOCSSINGLEWIRE)
	/* STM32 HDSEL internally joins TX/RX on the TX pin and releases the pin while
	 * receiving. Push-pull gives adequate rise time at 500 kbaud; the pull-up
	 * defines the idle state while neither side transmits. */
	const unsigned long single_wire_flags = single_wire
			? (SER_SINGLEWIRE_ENABLED | SER_SINGLEWIRE_PUSHPULL | SER_SINGLEWIRE_PULLUP)
			: 0;

	if (::ioctl(_uart_fd, TIOCSSINGLEWIRE, single_wire_flags) < 0) {
		PX4_ERR("TIOCSSINGLEWIRE %s failed (%i)", single_wire ? "enable" : "disable", errno);
		close_uart();
		return false;
	}

	_single_wire_active = single_wire;
#else
	if (single_wire) {
		PX4_ERR("single-wire UART is not supported on this target");
		close_uart();
		return false;
	}
#endif

	/* Re-read after TCSETS; avoid a second tcsetattr (can clear NuttX c_speed → line_baud=0). */
	struct termios verify {};
	tcgetattr(_uart_fd, &verify);
	_line_baud = (int)cfgetspeed(&verify);

	if (_line_baud <= 0) {
		_line_baud = baud;
		PX4_WARN("cfgetspeed returned 0; assuming %i bps", baud);

	} else if (_line_baud != baud) {
		PX4_WARN("UART line speed %i bps (requested %i)", _line_baud, baud);
	}

	PX4_INFO("UART open %s @ %i bps (line=%i, %s)", _device_path, baud, _line_baud,
		 _single_wire_active ? "single-wire TX pin" : "full-duplex adapter");

	/* fresh UART → drop any stale bytes and reset RX parser */
	fs_uart_servo::flush_rx(_uart_fd);
	_rx_parser.reset();
	_query_tx_us = 0;
	_online_flags = 0;

	for (auto &a : _fb_angle_deg) {
		a = NAN;
	}

	return true;
}

void FsUartServo::update_params(const bool force)
{
	if (!_parameter_update_sub.updated() && !force) {
		return;
	}

	parameter_update_s p{};
	_parameter_update_sub.copy(&p);

	updateParams();

	_last_sent_sample = 0;
	_sent_once = false;

	apply_service_schedule();

	const int want_baud = normalize_baud((_baud_cli > 0) ? _baud_cli : _param_fs_ut_baud.get());

	if (_uart_fd >= 0 && want_baud != _effective_baud) {
		if (!open_uart()) {
			PX4_WARN("UART reopen after baud update failed");
		}
	}
}

uint32_t FsUartServo::schedule_interval_us() const
{
	const int configured_hz = math::constrain(_param_fs_ut_r_hz.get(), (int32_t)20, (int32_t)400);

	/* 辨识自动回传、或 1 Mbaud 下常开 FS_UT_FB：需要在同步帧间隙收应答 */
	if (_iden_fb_active || (_param_fs_ut_fb.get() != 0 && uart_is_1mbaud())) {
		return k_high_rate_service_interval_us;
	}

	return 1_s / configured_hz;
}

void FsUartServo::apply_service_schedule()
{
	const uint32_t want = schedule_interval_us();

	if (want != _service_interval_us) {
		_service_interval_us = want;
		ScheduleOnInterval(_service_interval_us);
	} else if (_service_interval_us == 0) {
		_service_interval_us = want;
		ScheduleOnInterval(_service_interval_us);
	}
}

bool FsUartServo::uart_is_1mbaud() const
{
	const int baud = (_line_baud > 0) ? _line_baud
			 : normalize_baud((_baud_cli > 0) ? _baud_cli : _param_fs_ut_baud.get());
	return baud >= 1000000;
}

bool FsUartServo::feedback_enabled() const
{
	return (_param_fs_ut_fb.get() != 0) || _iden_fb_active;
}

uint32_t FsUartServo::iden_ctrl_hz() const
{
	return (uint32_t)math::constrain(_param_iden_sv_hz.get(), (int32_t)20, (int32_t)200);
}

hrt_abstime FsUartServo::iden_ctrl_interval_us() const
{
	return 1_s / iden_ctrl_hz();
}

uint32_t FsUartServo::feedback_target_hz_per_ch() const
{
	/* 辨识：只查目标舵机，与 IDEN_SV_HZ 发送同频 */
	if (_iden_fb_active) {
		return iden_ctrl_hz();
	}

	return uart_is_1mbaud() ? k_high_rate_feedback_hz_per_ch : k_legacy_feedback_hz_per_ch;
}

hrt_abstime FsUartServo::query_timeout_us() const
{
	return uart_is_1mbaud() ? k_high_rate_query_timeout_us : k_legacy_query_timeout_us;
}

void FsUartServo::refill_query_budget(hrt_abstime now)
{
	if (_next_query_budget_us == 0) {
		_next_query_budget_us = now;
	}

	if (now < _next_query_budget_us) {
		return;
	}

	const hrt_abstime window_us = _iden_fb_active ? iden_ctrl_interval_us() : k_control_interval_us;
	const uint32_t rate_hz = _iden_fb_active ? iden_ctrl_hz() : k_control_rate_hz;
	const uint64_t windows = (now - _next_query_budget_us) / window_us + 1U;
	_next_query_budget_us += windows * window_us;

	/* 每控制窗口贡献 n_ch*target/rate 个查询名额；余数留到下一窗。
	 * 辨识只查 1 路且 target=IDEN_SV_HZ → 每窗正好 1 次。 */
	const uint32_t n_ch = _iden_fb_active ? 1u : actuator_servos_s::NUM_CONTROLS;
	const uint32_t max_budget = _iden_fb_active ? 1u : k_max_query_budget;
	const uint64_t credit = (uint64_t)_query_credit
				+ windows * n_ch * feedback_target_hz_per_ch();
	const uint32_t grant = (uint32_t)(credit / rate_hz);
	_query_credit = (uint32_t)(credit % rate_hz);
	_query_budget = (uint8_t)math::min(max_budget, (uint32_t)_query_budget + grant);
}

void FsUartServo::send_latest_frame(const actuator_servos_s &sv)
{
	if (_uart_fd < 0) {
		return;
	}

	refresh_servo_kill_state();

	const float gain = _param_fs_ut_gain.get();
	const float trm = _servo_kill_zero ? 0.f : _param_fs_ut_trm.get();
	const bool mturn = _param_fs_ut_mturn.get() != 0;
	const float angle_max = mturn ? 368640.f : 180.f;
	const float angle_min = mturn ? -368640.f : -180.f;

	float deg[actuator_servos_s::NUM_CONTROLS] {};

	for (unsigned i = 0; i < actuator_servos_s::NUM_CONTROLS; i++) {
		const float ctrl = sv.control[i];

		if (!PX4_ISFINITE(ctrl)) {
			deg[i] = _last_deg[i];

		} else if (_servo_kill_zero) {
			deg[i] = 0.f;

		} else {
			deg[i] = math::constrain(ctrl * gain + trm, angle_min, angle_max);
		}
	}

	send_sync_deg(deg, sv.timestamp_sample, false);
}

void FsUartServo::send_iden_frame(uint8_t idx, float cmd_deg)
{
	float deg[actuator_servos_s::NUM_CONTROLS] {};

	if (!_servo_kill_zero && PX4_ISFINITE(cmd_deg)
	    && idx < actuator_servos_s::NUM_CONTROLS) {
		const bool mturn = _param_fs_ut_mturn.get() != 0;
		const float angle_max = mturn ? 368640.f : 180.f;
		const float angle_min = mturn ? -368640.f : -180.f;
		deg[idx] = math::constrain(cmd_deg, angle_min, angle_max);
	}

	send_sync_deg(deg, hrt_absolute_time(), true);
}

void FsUartServo::send_sync_deg(const float angle_deg[actuator_servos_s::NUM_CONTROLS],
			       uint64_t timestamp_sample, bool skip_sample_dedup)
{
	if (_uart_fd < 0) {
		return;
	}

	refresh_servo_kill_state();

	fs_uart_servo::SyncServoParam sp[actuator_servos_s::NUM_CONTROLS] {};

	auto u16_clamp_us = [](int32_t v) {
		return static_cast<uint16_t>(math::constrain(v, (int32_t)0, (int32_t)UINT16_MAX));
	};

	const uint16_t interval = u16_clamp_us(_param_fs_ut_intv.get());
	const bool fastest_mode = interval == 0;
	const uint16_t t_acc = u16_clamp_us(_param_fs_ut_tac.get());
	const uint16_t t_dec = u16_clamp_us(_param_fs_ut_tdc.get());
	const bool mturn = _param_fs_ut_mturn.get() != 0;

	for (unsigned i = 0; i < actuator_servos_s::NUM_CONTROLS; i++) {
		sp[i].id = static_cast<uint8_t>(i);
		sp[i].interval_ms = interval;
		sp[i].t_acc_ms = t_acc;
		sp[i].t_dec_ms = t_dec;
		sp[i].power_mw = 0;

		if (_servo_kill_zero) {
			sp[i].angle_deg = 0.f;
			_last_deg[i] = 0.f;

		} else {
			sp[i].angle_deg = angle_deg[i];
			_last_deg[i] = sp[i].angle_deg;
		}

		_last_cmd_deg[i] = sp[i].angle_deg;
	}

	uint8_t frame[256] {};
	uint16_t flen = 0;

	if (mturn && fastest_mode) {
		flen = fs_uart_servo::build_sync_angle_mturn_frame(sp, actuator_servos_s::NUM_CONTROLS, frame, sizeof(frame));

	} else if (mturn) {
		flen = fs_uart_servo::build_sync_angle_mturn_by_interval_frame(sp, actuator_servos_s::NUM_CONTROLS, frame,
				sizeof(frame));

	} else if (fastest_mode) {
		flen = fs_uart_servo::build_sync_angle_frame(sp, actuator_servos_s::NUM_CONTROLS, frame, sizeof(frame));

	} else {
		flen = fs_uart_servo::build_sync_angle_by_interval_frame(sp, actuator_servos_s::NUM_CONTROLS, frame,
				sizeof(frame));
	}

	if (flen == 0) {
		_tx_drop++;
		return;
	}

	/* 辨识按 IDEN_SV_HZ；平时同步帧仍封顶 200 Hz */
	const hrt_abstime min_send_us = skip_sample_dedup ? iden_ctrl_interval_us() : 5_ms;
	const bool kill_edge = _servo_kill_zero != _servo_kill_zero_prev;

	if (!skip_sample_dedup) {
		const bool new_sample = !_sent_once || (timestamp_sample != _last_sent_sample) || kill_edge;

		if (!new_sample) {
			if (_sent_once && timestamp_sample == _last_sent_sample) {
				_ca_drop_dup++;
			}

			return;
		}
	}

	const hrt_abstime now = hrt_absolute_time();
	static constexpr hrt_abstime k_send_tolerance_us = 200;

	if (_last_send != 0 && (now - _last_send) + k_send_tolerance_us < min_send_us) {
		_send_pending = true;
		return;
	}

	_send_pending = false;
	_last_send = now;
	ssize_t written = ::write(_uart_fd, frame, flen);

	if (written != (ssize_t)flen) {
		_tx_drop++;

		if (written < 0) {
			PX4_WARN("UART write errno %i (%s)", errno, strerror(errno));
		}

	} else {
		_tx_count++;
		_tx_bytes_win += (uint32_t)flen;
		_tx_frames_win++;
		_last_tx_len = flen;
		_last_sent_sample = timestamp_sample;
		_sent_once = true;
		_servo_kill_zero_prev = _servo_kill_zero;
		const uint32_t baud = (_line_baud > 0) ? (uint32_t)_line_baud : (uint32_t)_effective_baud;
		const hrt_abstime wire_time_us = ((uint64_t)flen * 10ULL * 1000000ULL + baud - 1ULL) / baud;
		const hrt_abstime quiet_us = (baud >= 1000000)
				? k_high_rate_post_control_quiet_us : k_legacy_post_control_quiet_us;
		_post_tx_until = now + wire_time_us + quiet_us;
	}
}

void FsUartServo::send_zero_sync_frame()
{
	actuator_servos_s srv{};
	srv.timestamp = hrt_absolute_time();
	srv.timestamp_sample = srv.timestamp;

	for (unsigned i = 0; i < actuator_servos_s::NUM_CONTROLS; i++) {
		srv.control[i] = 0.f;
	}

	/* 强制首帧：CA 未配置舵机时 topic 可能从未发布 */
	_sent_once = false;
	_last_sent_sample = 0;
	send_latest_frame(srv);
}

/* === FB query / RX state machine ================================================== */

void FsUartServo::disable_feedback()
{
	fs_uart_servo::flush_rx(_uart_fd);
	_rx_parser.reset();
	_query_tx_us = 0;
	_next_query_budget_us = 0;
	_query_credit = 0;
	_query_budget = 0;
	_online_flags = 0;

	for (auto &a : _fb_angle_deg) {
		a = NAN;
	}
}

void FsUartServo::enable_feedback()
{
	fs_uart_servo::flush_rx(_uart_fd);
	_rx_parser.reset();
	_query_tx_us = 0;
	_query_id = 0;
	_next_query_budget_us = hrt_absolute_time();
	_query_credit = 0;
	_query_budget = 0;
	_rtt_ema_us = 0.f;
	_rtt_max_us = 0.f;
}

void FsUartServo::advance_query_id()
{
	if (_iden_pin_query) {
		_query_id = _iden_servo_idx;
		return;
	}

	_query_id = (uint8_t)((_query_id + 1) % actuator_servos_s::NUM_CONTROLS);
}

servo_iden::ServoIdenSequencer::Config FsUartServo::make_iden_config() const
{
	servo_iden::ServoIdenSequencer::Config c{};
	c.f0 = _param_iden_ch_f0.get();
	c.f1 = _param_iden_ch_f1.get();
	c.dur_s = _param_iden_ch_dur.get();
	c.amp_deg = _param_iden_ch_amp.get();
	c.amin_deg = _param_iden_ch_amin.get();
	c.step_deg = _param_iden_ch_step.get();
	c.rep = static_cast<int>(_param_iden_ch_rep.get());
	c.half_run = _param_iden_ch_half.get() != 0;
	return c;
}

void FsUartServo::fill_iden_feedback(servo_iden::ServoIdenSequencer::Feedback &fb)
{
	fb = {};

	if (_iden_servo_idx < actuator_servos_s::NUM_CONTROLS
	    && PX4_ISFINITE(_fb_angle_deg[_iden_servo_idx])) {
		fb.angle_deg = _fb_angle_deg[_iden_servo_idx];
		fb.angle_valid = true;
	}

	vehicle_angular_velocity_s w{};

	if (_vehicle_angular_velocity_sub.copy(&w)) {
		_gyro_dps[0] = math::degrees(w.xyz[0]);
		_gyro_dps[1] = math::degrees(w.xyz[1]);
		_gyro_dps[2] = math::degrees(w.xyz[2]);
		fb.gyro_dps = sqrtf(_gyro_dps[0] * _gyro_dps[0]
				    + _gyro_dps[1] * _gyro_dps[1]
				    + _gyro_dps[2] * _gyro_dps[2]);
		fb.gyro_valid = PX4_ISFINITE(fb.gyro_dps);

	} else {
		_gyro_dps[0] = _gyro_dps[1] = _gyro_dps[2] = NAN;
	}
}

void FsUartServo::publish_servo_iden_data(const servo_iden::ServoIdenSequencer::Output &out,
		uint8_t servo_idx, hrt_abstime tx_time)
{
	Servo_iden_data_s data{};
	data.timestamp = tx_time;
	data.iden_active = !out.done && (out.phase != servo_iden::ServoIdenSequencer::Idle);
	data.servo_idx = servo_idx;
	data.chirp_signal = out.cmd_deg;
	data.cmd_deg = out.cmd_deg;
	data.phase = out.phase;
	data.rep_idx = out.rep_idx;
	data.sweep_t = out.sweep_t;
	data.inst_freq_hz = out.inst_freq_hz;
	data.amp_deg = out.amp_deg;
	data.vmax_dps = out.vmax_dps;
	data.f1_eff_hz = out.f1_eff_hz;
	data.fb_source = out.fb_source;
	data.gyro_dps[0] = _gyro_dps[0];
	data.gyro_dps[1] = _gyro_dps[1];
	data.gyro_dps[2] = _gyro_dps[2];

	/* 发送时刻把 EKF 姿态写成滚转/俯仰（度），后处理不必再记 vehicle_attitude */
	vehicle_attitude_s att{};

	if (_vehicle_attitude_sub.copy(&att)) {
		matrix::Quatf q(att.q);
		matrix::Eulerf euler(q);
		data.roll_angle = math::degrees(euler.phi());
		data.pitch_angle = math::degrees(euler.theta());

	} else {
		data.roll_angle = NAN;
		data.pitch_angle = NAN;
	}

	const bool fb_on = feedback_enabled();

	if (fb_on && servo_idx < actuator_servos_s::NUM_CONTROLS) {
		data.fb_angle = _fb_angle_deg[servo_idx];

	} else {
		data.fb_angle = NAN;
	}

	_servo_iden_data_pub.publish(data);
}

void FsUartServo::on_query_timeout()
{
	/* 标记本通道掉线、下一通道 */
	_online_flags &= ~(1u << _query_id);
	_fb_angle_deg[_query_id] = NAN;
	_query_err_total++;
	advance_query_id();
	_query_tx_us = 0;
	_rx_parser.reset();
}

bool FsUartServo::maybe_send_query()
{
	if (_uart_fd < 0) {
		return false;
	}

	if (!feedback_enabled()) {
		return false;
	}

	/* 等待中：让 poll_rx 收完或超时 */
	if (_query_tx_us != 0) {
		return false;
	}

	const hrt_abstime now = hrt_absolute_time();

	/* 同步帧结束后的安静期，以及当前控制窗口的查询配额。 */
	if (now < _post_tx_until || _query_budget == 0) {
		return false;
	}

	/* 1 Mbaud：下一帧同步指令前留出超时窗口，让未到的应答先结束，避免和发送撞车。 */
	if (uart_is_1mbaud() && _last_send != 0) {
		const hrt_abstime ctrl_dt = _iden_fb_active ? iden_ctrl_interval_us() : k_control_interval_us;
		const hrt_abstime next_control = _last_send + ctrl_dt;

		if (now < next_control && now + k_query_control_guard_us >= next_control) {
			return false;
		}
	}

	if (_iden_pin_query) {
		_query_id = _iden_servo_idx;
	}

	const bool mturn = _param_fs_ut_mturn.get() != 0;
	uint8_t frame[8];
	uint16_t flen = mturn
			? fs_uart_servo::build_query_angle_mturn_frame(_query_id, frame, sizeof(frame))
			: fs_uart_servo::build_query_angle_frame(_query_id, frame, sizeof(frame));

	if (flen == 0) {
		return false;
	}

	const ssize_t w = ::write(_uart_fd, frame, flen);

	if (w == (ssize_t)flen) {
		_tx_bytes_win += (uint32_t)flen;
		_query_tx_us = now;
		_query_budget--;
		_rx_parser.reset();
		return true;

	} else {
		/* 写失败：不计查询响应错误，下次再试。 */
		_tx_drop++;
		_query_tx_us = 0;
		return false;
	}
}

void FsUartServo::poll_rx()
{
	if (_uart_fd < 0) {
		return;
	}

	uint8_t buf[64];

	while (true) {
		const ssize_t n = ::read(_uart_fd, buf, sizeof(buf));

		if (n <= 0) {
			break;
		}

		_rx_bytes_win += (uint32_t)n;

		for (ssize_t i = 0; i < n; i++) {
			fs_uart_servo::ResponseParser::Frame f{};
			const auto r = _rx_parser.feed(buf[i], f);

			if (r == fs_uart_servo::ResponseParser::Result::Frame_ok) {
				_rx_frame_total++;
				_last_rx_cmd = f.cmd_id;
				_last_rx_size = f.content_size;
				_last_rx_id = (f.content_size > 0) ? f.content[0] : UINT8_MAX;

				if (_last_rx_id < actuator_servos_s::NUM_CONTROLS) {
					_rx_seen_id_flags |= (1u << _last_rx_id);
				}

				/* Request echoes have header 0x12 0x4c and never reach Frame_ok. For a
				 * response, require an outstanding query and an exact cmd/id match so
				 * delayed or duplicate frames cannot advance the query ring. */
				const bool mturn = _param_fs_ut_mturn.get() != 0;
				const uint8_t expected_cmd = mturn
						? fs_uart_servo::k_cmd_query_angle_mturn
						: fs_uart_servo::k_cmd_query_angle;
				const uint8_t min_content_size = mturn ? 5 : 3;

				const bool response_matches = _query_tx_us != 0 && f.cmd_id == expected_cmd
				    && f.content_size >= min_content_size && f.content[0] == _query_id;

				if (response_matches) {
					const uint8_t id = f.content[0];
					float angle_deg = NAN;

					if (f.cmd_id == fs_uart_servo::k_cmd_query_angle && f.content_size >= 3) {
						const int16_t raw = (int16_t)(f.content[1] | (f.content[2] << 8));
						angle_deg = 0.1f * (float)raw;

					} else if (f.cmd_id == fs_uart_servo::k_cmd_query_angle_mturn && f.content_size >= 5) {
						const int32_t raw = (int32_t)((uint32_t)f.content[1]
									      | ((uint32_t)f.content[2] << 8)
									      | ((uint32_t)f.content[3] << 16)
									      | ((uint32_t)f.content[4] << 24));
						angle_deg = 0.1f * (float)raw;
					}

					if (PX4_ISFINITE(angle_deg)) {
						_fb_angle_deg[id] = angle_deg;
						_online_flags |= (1u << id);
						_query_ok_total++;
						_query_ok_win++;
						_query_ok_per_ch_win[id]++;

						if (_query_tx_us != 0) {
							const float dt_us = (float)(hrt_absolute_time() - _query_tx_us);
							_rtt_ema_us = (_rtt_ema_us > 0.f) ? (0.9f * _rtt_ema_us + 0.1f * dt_us) : dt_us;

							if (dt_us > _rtt_max_us) {
								_rtt_max_us = dt_us;
							}
						}

						/* 切下一通道（辨识时钉在目标 ID） */
						advance_query_id();
						_query_tx_us = 0;
					}
				} else {
					_rx_match_miss_total++;
				}

			} else if (r == fs_uart_servo::ResponseParser::Result::Checksum_err && _query_tx_us != 0) {
				/* Header/cmd/size misses while scanning are commonly bytes from our
				 * echoed request frames. Only a checksum failure on a candidate response
				 * is a receive error; an unanswered request is counted once on timeout. */
				_query_err_total++;
				_rx_checksum_err_total++;
			}
		}
	}

	/* 超时检测 */
	if (_query_tx_us != 0) {
		const hrt_abstime now = hrt_absolute_time();

		if (now - _query_tx_us > query_timeout_us()) {
			on_query_timeout();
		}
	}
}

/* === Stats & Fs_data ============================================================== */

void FsUartServo::update_stats_window()
{
	const hrt_abstime now = hrt_absolute_time();

	if (_stats_t0 == 0) {
		_stats_t0 = now;
		return;
	}

	if (now - _stats_t0 < 1_s) {
		return;
	}

	const float dt = (float)(now - _stats_t0) * 1e-6f;
	const float baud_f = (_line_baud > 0) ? (float)_line_baud : 0.f;

	_tx_bps = (float)_tx_bytes_win * 10.f / dt; /* 8N1 ≈ 10 bit/byte */
	_rx_bps = (float)_rx_bytes_win * 10.f / dt;
	_bus_util_pct = (baud_f > 0.f) ? (_tx_bps + _rx_bps) / baud_f * 100.f : 0.f;
	_tx_hz = (float)_tx_frames_win / dt;
	_ca_hz = (float)_ca_samples_win / dt;
	_effective_ctrl_hz = math::min(_tx_hz, _ca_hz);

	for (unsigned i = 0; i < actuator_servos_s::NUM_CONTROLS; i++) {
		_query_hz_per_ch[i] = (float)_query_ok_per_ch_win[i] / dt;
		_query_ok_per_ch_win[i] = 0;
	}

	_tx_bytes_win = 0;
	_rx_bytes_win = 0;
	_tx_frames_win = 0;
	_ca_samples_win = 0;
	_query_ok_win = 0;
	_rtt_max_us = 0.f;
	_stats_t0 = now;
}

void FsUartServo::publish_fs_data()
{
	const hrt_abstime now = hrt_absolute_time();

	/* Fs_data 与控制节拍对齐（平时 200 Hz；辨识跟 IDEN_SV_HZ）。 */
	const hrt_abstime fs_pub_dt = _iden_fb_active ? iden_ctrl_interval_us() : k_control_interval_us;

	if (_last_fs_data_pub != 0 && (now - _last_fs_data_pub) < fs_pub_dt) {
		return;
	}

	_last_fs_data_pub = now;

	Fs_data_s msg{};
	msg.timestamp_sample = _last_srv_valid ? _last_srv.timestamp_sample : 0;

	for (unsigned i = 0; i < actuator_servos_s::NUM_CONTROLS; i++) {
		msg.cmd_deg[i] = _last_cmd_deg[i];
	}

	msg.mturn_mode = (uint8_t)(_param_fs_ut_mturn.get() != 0 ? 1 : 0);
	msg.kill_zero = (uint8_t)(_servo_kill_zero ? 1 : 0);
	msg.line_baud = (int32_t)_line_baud;
	msg.last_tx_len = _last_tx_len;

	msg.tx_bps = _tx_bps;
	msg.rx_bps = _rx_bps;
	msg.bus_util_pct = _bus_util_pct;
	msg.tx_frame_hz = _tx_hz;
	msg.ca_sample_hz = _ca_hz;
	msg.effective_ctrl_hz = _effective_ctrl_hz;
	msg.tx_drop = _tx_drop;
	msg.ca_drop_dup = _ca_drop_dup;

	const bool fb_on = feedback_enabled();
	msg.fb_enabled = (uint8_t)(fb_on ? 1 : 0);

	for (unsigned i = 0; i < actuator_servos_s::NUM_CONTROLS; i++) {
		msg.fb_angle_deg[i] = fb_on ? _fb_angle_deg[i] : NAN;
		msg.fb_query_hz[i] = fb_on ? _query_hz_per_ch[i] : 0.f;
	}

	msg.online_flags = fb_on ? _online_flags : (uint8_t)0;
	msg.query_ok = _query_ok_total;
	msg.query_err = _query_err_total;
	msg.rtt_ema_us = fb_on ? _rtt_ema_us : 0.f;
	msg.rtt_max_us = fb_on ? _rtt_max_us : 0.f;

	msg.timestamp = hrt_absolute_time();
	_fs_data_pub.publish(msg);
}

void FsUartServo::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	update_params(false);
	refresh_servo_kill_state();

	const bool uart_was_closed = (_uart_fd < 0);

	if (_uart_fd < 0) {
		if (!open_uart()) {
			// No spam: UART may be wired later
			return;
		}
	}

	if (uart_was_closed) {
		send_zero_sync_frame();
	}

	actuator_servos_s srv{};

	/* 先缓存最新指令，UART 真正占用要等 RX 排空后再仲裁，避免 200 Hz 发送把回传饿死。 */
	while (_actuator_servos_sub.update(&srv)) {
		_last_srv = srv;
		_last_srv_valid = true;
		_ca_samples_win++;
	}

	if (!_last_srv_valid && _actuator_servos_sub.copy(&_last_srv)) {
		_last_srv_valid = true;
		_ca_samples_win++;
	}

	Servo_iden_ctrl_s iden_ctrl{};
	const bool have_ctrl = _servo_iden_ctrl_sub.copy(&iden_ctrl);
	const bool iden_enable = have_ctrl && iden_ctrl.enable;
	_iden_servo_idx = (have_ctrl && iden_ctrl.servo_idx < actuator_servos_s::NUM_CONTROLS)
			  ? iden_ctrl.servo_idx : (uint8_t)0;

	actuator_armed_s armed_msg{};
	const bool is_armed = _actuator_armed_sub.copy(&armed_msg) && armed_msg.armed;

	if (_iden_was_armed && !is_armed) {
		_iden_completed_this_arm = false;
		_iden_seq.reset();
		_iden_last_out = {};
		_iden_pin_query = false;
	}

	_iden_was_armed = is_armed;

	if (!iden_enable) {
		_iden_seq.reset();
		_iden_pin_query = false;

	} else if (!_iden_completed_this_arm) {
		_iden_pin_query = true;
		_query_id = _iden_servo_idx;
	}

	const bool iden_hold = iden_enable && !_servo_kill_zero;
	const bool iden_run = iden_hold && !_iden_completed_this_arm;

	/* 扫频进行中自动开回传；结束后若 FS_UT_FB=0 则关掉 */
	_iden_fb_active = iden_run;

	const bool fb_now = feedback_enabled();

	if (fb_now != _fb_enabled_prev) {
		if (fb_now) {
			enable_feedback();

		} else {
			disable_feedback();
		}

		_fb_enabled_prev = fb_now;
	}

	/* enable_feedback() 会把查询 ID 置 0，辨识需钉回目标舵机 */
	if (_iden_pin_query) {
		_query_id = _iden_servo_idx;
	}

	apply_service_schedule();

	/* 回传：必须先收 RX/处理超时，再决定本周期总线由查询还是同步帧使用。 */
	if (fb_now) {
		poll_rx();
		refill_query_budget(hrt_absolute_time());

		if (_iden_fb_active && _query_budget > 1) {
			_query_budget = 1;
		}
	}

	const bool kill_edge = _servo_kill_zero != _servo_kill_zero_prev;
	const hrt_abstime now = hrt_absolute_time();
	const bool query_due = fb_now && _query_tx_us == 0 && _query_budget > 0 && now >= _post_tx_until;

	if (iden_hold) {
		const bool control_due = (_last_send == 0 || kill_edge
					  || (now - _last_send) + 200 >= iden_ctrl_interval_us());

		if (_query_tx_us == 0) {
			if (kill_edge) {
				send_iden_frame(_iden_servo_idx, 0.f);

			} else if (control_due) {
				if (iden_run) {
					servo_iden::ServoIdenSequencer::Feedback fb{};
					fill_iden_feedback(fb);
					const auto out = _iden_seq.update(now, make_iden_config(), fb);
					_iden_last_out = out;
					const hrt_abstime send_before = _last_send;
					send_iden_frame(_iden_servo_idx, out.cmd_deg);

					if (_last_send != send_before) {
						publish_servo_iden_data(out, _iden_servo_idx, _last_send);
					}

					if (out.done) {
						_iden_completed_this_arm = true;
						_iden_pin_query = false;
					}

				} else {
					/* 已完成：继续发 0° 并发布 Done，供 CA 锁存（保持到本次解锁结束） */
					send_iden_frame(_iden_servo_idx, 0.f);
					servo_iden::ServoIdenSequencer::Output done_out = _iden_last_out;
					done_out.cmd_deg = 0.f;
					done_out.phase = servo_iden::ServoIdenSequencer::Done;
					done_out.done = true;
					done_out.amp_deg = 0.f;
					done_out.inst_freq_hz = 0.f;
					publish_servo_iden_data(done_out, _iden_servo_idx, _last_send);
				}

			} else if (query_due && iden_run) {
				(void)maybe_send_query();
			}
		}

	} else {
		const bool sync_pending = _last_srv_valid
				&& (!_sent_once || _send_pending || kill_edge || _last_srv.timestamp_sample != _last_sent_sample);
		const bool control_due = sync_pending && (_last_send == 0 || kill_edge
				|| (now - _last_send) + 200 >= k_control_interval_us);

		if (_query_tx_us == 0) {
			if (kill_edge && sync_pending) {
				/* Safety zero always wins over telemetry. */
				send_latest_frame(_last_srv);

			} else if (control_due) {
				send_latest_frame(_last_srv);

			} else if (query_due) {
				(void)maybe_send_query();
			}
		}
	}

	update_stats_window();
	publish_fs_data();
}

int FsUartServo::task_spawn(int argc, char *argv[])
{
	const char *dev = nullptr;
	int baud_override = -1;

	int myoptind = 1;
	int ch{};
	const char *myoptarg{nullptr};

	while ((ch = px4_getopt(argc, argv, "d:b:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {

		case 'd':
			dev = myoptarg;
			break;

		case 'b':
			baud_override = atoi(myoptarg);
			break;

		default:
			print_usage(nullptr);
			return PX4_ERROR;
		}
	}

	char device_path[64] {};
	const char *use_dev = dev;

	if (use_dev == nullptr || strlen(use_dev) == 0) {
		use_dev = default_uart_device_path();
	}

	strncpy(device_path, use_dev, sizeof(device_path) - 1);

	FsUartServo *const inst = new FsUartServo(device_path, baud_override);

	if (!inst) {
		PX4_ERR("alloc failed");
		return PX4_ERROR;
	}

	_object.store(inst);
	_task_id = task_id_is_work_queue;

	if (!inst->init()) {
		delete inst;
		_object.store(nullptr);
		_task_id = -1;
		return PX4_ERROR;
	}

	return PX4_OK;
}

int FsUartServo::custom_command(int argc, char *argv[])
{
	return print_usage("unsupported command");
}

int FsUartServo::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s", reason);
	}

	PRINT_MODULE_DESCRIPTION(R"DESCR_STR(
### Description

Subscribes to `actuator_servos`. Interprets `control[0–7]` as **tilt angles in degrees**
(from `control_allocator` / custom allocation) plus optional scaling/trim (`FS_UT_GAIN`, `FS_UT_TRM`).

Translates FashionStar **sync angle** commands (FashionStar cmd ID 25) and writes to a UART.
`FS_UT_INTV=0` selects the official fastest/basic command; a non-zero value selects
the angle-by-interval command with `FS_UT_TAC`/`FS_UT_TDC` profiling.
`FS_UT_MTURN` selects single-turn (±180°) or multi-turn (±368640°) subcommand layout.
Do not simultaneously map `Servo1–8` to PWM outputs for the same angles (degrees are not PX4 −1…1 servo norm).

Defaults: uart `CONFIG_BOARD_SERIAL_TEL2` if defined, baud `FS_UT_BAUD` (115200), `FS_UT_INTV` (500 ms).
Transmits on new actuator_servos samples only; bus capped ~200 Hz to avoid re-triggering motion.

### Boot

`control_allocator` starts this driver when **IDEN_TYPE=3** (bus servo identify); stops it for all other modes.
Ensure MAVLink is not using the same UART (default board TELEM2).

### CLI

Starts with `[ -d uart ] [ -b baud ]` overriding params for this process.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("fs_uart_servo", "driver");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start FashionStar servo driver");

	PRINT_MODULE_USAGE_PARAM_COMMENT("Optional `-d dev` selects UART device; default board TELEM2 string.");
	PRINT_MODULE_USAGE_PARAM_COMMENT("Optional `-b baud` selects baud; FS_UT_BAUD otherwise.");

	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int FsUartServo::print_status()
{
	hrt_abstime now = hrt_absolute_time();

	PX4_INFO("UART %s fd=%i param_baud=%i line_baud=%i wire=%s", _device_path, _uart_fd, _effective_baud, _line_baud,
		 _single_wire_active ? "single(TX pin)" : "adapter(TX/RX)");
	PX4_INFO("TX %" PRIu32 " drop %" PRIu32 " last %" PRIu32 " us ago",
		 _tx_count, _tx_drop,
		 (uint32_t)(_last_send == 0 ? 999999999u : (now > _last_send ? (uint32_t)(now - _last_send) : 0u)));

	PX4_INFO("mode %s (FS_UT_MTURN=%lli)",
		 _param_fs_ut_mturn.get() != 0 ? "Multi-turn" : "Single-turn",
		 (long long)_param_fs_ut_mturn.get());

	PX4_INFO("gain %.4f trim %.2f motion=%s intv %lld ms tac %lld tdc %lld poll %llu Hz service %.0f Hz",
		 (double)_param_fs_ut_gain.get(),
		 (double)_param_fs_ut_trm.get(),
		 _param_fs_ut_intv.get() == 0 ? "fast" : "timed",
		 (long long)_param_fs_ut_intv.get(),
		 (long long)_param_fs_ut_tac.get(),
		 (long long)_param_fs_ut_tdc.get(),
		 (unsigned long long)_param_fs_ut_r_hz.get(),
		 _service_interval_us > 0 ? 1000000.0 / (double)_service_interval_us : 0.0);

	/* 总线统计（始终显示）*/
	PX4_INFO("TX bps %.0f (~%.1f kbps) Hz %.1f  drop_dup %" PRIu32,
		 (double)_tx_bps, (double)(_tx_bps / 1000.0f), (double)_tx_hz, _ca_drop_dup);
	PX4_INFO("RX bps %.0f (~%.1f kbps)  util %.2f%%",
		 (double)_rx_bps, (double)(_rx_bps / 1000.0f), (double)_bus_util_pct);
	PX4_INFO("CA samples %.1f Hz   effective ctrl %.1f Hz (min(tx,ca))",
		 (double)_ca_hz, (double)_effective_ctrl_hz);
	PX4_INFO("iden ctrl/fb %" PRIu32 " Hz (IDEN_SV_HZ)", iden_ctrl_hz());

	if (feedback_enabled()) {
		PX4_INFO("feedback %s target=%" PRIu32 " Hz%s budget=%u online=0x%02X q_ok=%" PRIu32 " q_err=%" PRIu32,
			 _param_fs_ut_fb.get() != 0 ? "ON" : "AUTO(iden)",
			 feedback_target_hz_per_ch(),
			 _iden_fb_active ? "/ch ident-pin" : "/ch",
			 _query_budget, _online_flags, _query_ok_total, _query_err_total);
		PX4_INFO("RX frames=%" PRIu32 " mismatch=%" PRIu32 " checksum=%" PRIu32
			 " seen_id=0x%02X last(cmd/id/len)=%u/%u/%u waiting_id=%u",
			 _rx_frame_total, _rx_match_miss_total, _rx_checksum_err_total, _rx_seen_id_flags,
			 _last_rx_cmd, _last_rx_id, _last_rx_size, _query_id);
		PX4_INFO("per-ch Hz: %.1f %.1f %.1f %.1f %.1f %.1f %.1f %.1f",
			 (double)_query_hz_per_ch[0], (double)_query_hz_per_ch[1],
			 (double)_query_hz_per_ch[2], (double)_query_hz_per_ch[3],
			 (double)_query_hz_per_ch[4], (double)_query_hz_per_ch[5],
			 (double)_query_hz_per_ch[6], (double)_query_hz_per_ch[7]);
		PX4_INFO("angles[deg]: %.1f %.1f %.1f %.1f %.1f %.1f %.1f %.1f",
			 (double)_fb_angle_deg[0], (double)_fb_angle_deg[1],
			 (double)_fb_angle_deg[2], (double)_fb_angle_deg[3],
			 (double)_fb_angle_deg[4], (double)_fb_angle_deg[5],
			 (double)_fb_angle_deg[6], (double)_fb_angle_deg[7]);
		PX4_INFO("RTT ema=%.0f us  max=%.0f us  (timeout=%llu us)",
			 (double)_rtt_ema_us, (double)_rtt_max_us,
			 (unsigned long long)query_timeout_us());

	} else {
		PX4_INFO("feedback OFF（平时关；辨识扫频时自动按 IDEN_SV_HZ 回传）");
	}

	return 0;
}

extern "C" __EXPORT int fs_uart_servo_main(int argc, char *argv[])
{
	return FsUartServo::main(argc, argv);
}

/* 供 control_allocator 高频轮询的静默运行状态查询：
 * 不能用 "status" 子命令，它会打印整页状态，200 Hz 调用会把 CA 拖慢到几 Hz。 */
extern "C" __EXPORT bool fs_uart_servo_is_running(void)
{
	return FsUartServo::is_running();
}
