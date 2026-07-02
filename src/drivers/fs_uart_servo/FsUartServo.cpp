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

	const int hz = math::constrain(_param_fs_ut_r_hz.get(), (int32_t)20, (int32_t)400);
	ScheduleOnInterval(1_s / hz);
	/* UART is opened in Run() on the serial work queue (same task as write). */
	PX4_INFO("FashionStar servo: %s poll=%i Hz", _device_path, hz);

	return true;
}

void FsUartServo::close_uart()
{
	if (_uart_fd >= 0) {
		::close(_uart_fd);
		_uart_fd = -1;
	}

	_line_baud = -1;
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

	PX4_INFO("UART open %s @ %i bps (line=%i)", _device_path, baud, _line_baud);

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

	const int hz = math::constrain(_param_fs_ut_r_hz.get(), (int32_t)20, (int32_t)400);
	ScheduleOnInterval(1_s / hz);

	const int want_baud = normalize_baud((_baud_cli > 0) ? _baud_cli : _param_fs_ut_baud.get());

	if (_uart_fd >= 0 && want_baud != _effective_baud) {
		if (!open_uart()) {
			PX4_WARN("UART reopen after baud update failed");
		}
	}
}

void FsUartServo::send_latest_frame(const actuator_servos_s &sv)
{
	if (_uart_fd < 0) {
		return;
	}

	refresh_servo_kill_state();

	fs_uart_servo::SyncServoParam sp[actuator_servos_s::NUM_CONTROLS] {};

	const float gain = _param_fs_ut_gain.get();
	const float trm = _servo_kill_zero ? 0.f : _param_fs_ut_trm.get();

	auto u16_clamp_us = [](int32_t v) {
		return static_cast<uint16_t>(math::constrain(v, (int32_t)0, (int32_t)UINT16_MAX));
	};

	const bool mturn = _param_fs_ut_mturn.get() != 0;
	const float angle_max = mturn ? 368640.f : 180.f;
	const float angle_min = mturn ? -368640.f : -180.f;

	/* SDK examples use 500–1000 ms; interval=0 is not used on wire and may be ignored by servos. */
	uint16_t interval = u16_clamp_us(_param_fs_ut_intv.get());

	if (interval == 0) {
		interval = 500;
	}
	const uint16_t t_acc = u16_clamp_us(_param_fs_ut_tac.get());
	const uint16_t t_dec = u16_clamp_us(_param_fs_ut_tdc.get());

	for (unsigned i = 0; i < actuator_servos_s::NUM_CONTROLS; i++) {
		const float ctrl = sv.control[i];
		sp[i].id = static_cast<uint8_t>(i);
		sp[i].interval_ms = interval;
		sp[i].t_acc_ms = t_acc;
		sp[i].t_dec_ms = t_dec;
		sp[i].power_mw = 0;

		if (!PX4_ISFINITE(ctrl)) {
			sp[i].angle_deg = _last_deg[i];

		} else if (_servo_kill_zero) {
			sp[i].angle_deg = 0.f;
			_last_deg[i] = 0.f;

		} else {
			sp[i].angle_deg = ctrl * gain + trm;

			if (sp[i].angle_deg > angle_max) {
				sp[i].angle_deg = angle_max;

			} else if (sp[i].angle_deg < angle_min) {
				sp[i].angle_deg = angle_min;
			}

			_last_deg[i] = sp[i].angle_deg;
		}

		_last_cmd_deg[i] = sp[i].angle_deg;
	}

	uint8_t frame[256] {};
	uint16_t flen = 0;

	if (mturn) {
		flen = fs_uart_servo::build_sync_angle_mturn_by_interval_frame(sp, actuator_servos_s::NUM_CONTROLS, frame,
				sizeof(frame));

	} else {
		flen = fs_uart_servo::build_sync_angle_by_interval_frame(sp, actuator_servos_s::NUM_CONTROLS, frame,
				sizeof(frame));
	}

	if (flen == 0) {
		_tx_drop++;
		return;
	}

	/* 同步“按周期”：同一 CA 样本在 200Hz 轮询下重复 write 会不断重启轨迹 → 抽搐。
	 * 按 timestamp_sample 去重；CA 新样本才发。总线封顶 200 Hz。 */
	static constexpr hrt_abstime k_min_send_interval_us = 5_ms;

	const bool kill_edge = _servo_kill_zero != _servo_kill_zero_prev;
	_servo_kill_zero_prev = _servo_kill_zero;

	const bool new_sample = !_sent_once || (sv.timestamp_sample != _last_sent_sample) || kill_edge;

	if (!new_sample) {
		if (_sent_once && sv.timestamp_sample == _last_sent_sample) {
			_ca_drop_dup++;
		}

		return;
	}

	/* CA 真实节拍：sample 变化才算 */
	if (_sent_once && sv.timestamp_sample != _last_sent_sample) {
		_ca_samples_win++;

	} else if (!_sent_once) {
		_ca_samples_win++;
	}

	const hrt_abstime now = hrt_absolute_time();

	/* 容差 200μs：_last_send 记录在 write() 前，实际线上帧间隔还要加
	 * TX 时间(96B @500kbps ≈ 1.92ms)，真实间隔比软件计算值大。 */
	static constexpr hrt_abstime k_send_tolerance_us = 200;

	if (_last_send != 0 && (now - _last_send) + k_send_tolerance_us < k_min_send_interval_us) {
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
		_last_sent_sample = sv.timestamp_sample;
		_sent_once = true;
	}

	// _last_send = hrt_absolute_time();
	/* 2ms quiet period before issuing the next query, to let servos process */
	_post_tx_until = _last_send + 2_ms;
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

static constexpr hrt_abstime k_query_timeout_us = 8_ms;

void FsUartServo::disable_feedback()
{
	fs_uart_servo::flush_rx(_uart_fd);
	_rx_parser.reset();
	_query_tx_us = 0;
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
	_rtt_ema_us = 0.f;
	_rtt_max_us = 0.f;
}

void FsUartServo::on_query_timeout()
{
	/* 标记本通道掉线、下一通道 */
	_online_flags &= ~(1u << _query_id);
	_fb_angle_deg[_query_id] = NAN;
	_query_err_total++;
	_query_id = (uint8_t)((_query_id + 1) % actuator_servos_s::NUM_CONTROLS);
	_query_tx_us = 0;
	_rx_parser.reset();
}

void FsUartServo::maybe_send_query()
{
	if (_uart_fd < 0) {
		return;
	}

	if (_param_fs_ut_fb.get() == 0) {
		return;
	}

	/* 等待中：让 poll_rx 收完或超时 */
	if (_query_tx_us != 0) {
		return;
	}

	const hrt_abstime now = hrt_absolute_time();

	/* 紧跟同步帧后保留 2 ms 安静期 */
	if (now < _post_tx_until) {
		return;
	}

	const bool mturn = _param_fs_ut_mturn.get() != 0;
	uint8_t frame[8];
	uint16_t flen = mturn
			? fs_uart_servo::build_query_angle_mturn_frame(_query_id, frame, sizeof(frame))
			: fs_uart_servo::build_query_angle_frame(_query_id, frame, sizeof(frame));

	if (flen == 0) {
		return;
	}

	const ssize_t w = ::write(_uart_fd, frame, flen);

	if (w == (ssize_t)flen) {
		_tx_bytes_win += (uint32_t)flen;
		_query_tx_us = now;
		_rx_parser.reset();

	} else {
		/* 写失败：不计 ok，下次再试 */
		_query_tx_us = 0;
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
				/* content[0]=servo_id；后续 2/4 字节为角度 little-endian ×10 */
				if (f.content_size >= 3) {
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

					if (id < actuator_servos_s::NUM_CONTROLS && PX4_ISFINITE(angle_deg)) {
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

						/* 切下一通道 */
						_query_id = (uint8_t)((_query_id + 1) % actuator_servos_s::NUM_CONTROLS);
						_query_tx_us = 0;
					}
				}

			} else if (r == fs_uart_servo::ResponseParser::Result::Checksum_err
				   || r == fs_uart_servo::ResponseParser::Result::Cmd_err
				   || r == fs_uart_servo::ResponseParser::Result::Header_err
				   || r == fs_uart_servo::ResponseParser::Result::Size_err) {
				_query_err_total++;
			}
		}
	}

	/* 超时检测 */
	if (_query_tx_us != 0) {
		const hrt_abstime now = hrt_absolute_time();

		if (now - _query_tx_us > k_query_timeout_us) {
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

	/* 10 Hz */
	if (_last_fs_data_pub != 0 && (now - _last_fs_data_pub) < 100_ms) {
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

	const bool fb_on = (_param_fs_ut_fb.get() != 0);
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

	/* FS_UT_FB 边沿 */
	const bool fb_now = (_param_fs_ut_fb.get() != 0);

	if (fb_now != _fb_enabled_prev) {
		if (fb_now) {
			enable_feedback();

		} else {
			disable_feedback();
		}

		_fb_enabled_prev = fb_now;
	}

	actuator_servos_s srv{};

	while (_actuator_servos_sub.update(&srv)) {
		_last_srv = srv;
		_last_srv_valid = true;
		send_latest_frame(srv);
	}

	if (!_last_srv_valid && _actuator_servos_sub.copy(&_last_srv)) {
		_last_srv_valid = true;
		send_latest_frame(_last_srv);
	}

	if (_send_pending && _last_srv_valid) {
		send_latest_frame(_last_srv);
	}

	/* 回传：先收 RX，再决定要不要发下一个查询 */
	if (fb_now) {
		poll_rx();
		maybe_send_query();
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

Translates FashionStar **sync angle-by-interval** command (FashionStar cmd ID 25) and writes to a UART.
`FS_UT_MTURN` selects single-turn (±180°) or multi-turn (±368640°) subcommand layout.
Do not simultaneously map `Servo1–8` to PWM outputs for the same angles (degrees are not PX4 −1…1 servo norm).

Defaults: uart `CONFIG_BOARD_SERIAL_TEL2` if defined, baud `FS_UT_BAUD` (115200), `FS_UT_INTV` (500 ms).
Transmits on new actuator_servos samples only; bus capped ~200 Hz to avoid re-triggering motion.

### Boot

Set `FS_UT_BOOT=1` (multicopter, `SYS_CTRL_ALLOC=1`) after disabling MAVLink on the chosen uart.

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

	PX4_INFO("UART %s fd=%i param_baud=%i line_baud=%i", _device_path, _uart_fd, _effective_baud, _line_baud);
	PX4_INFO("TX %" PRIu32 " drop %" PRIu32 " last %" PRIu32 " us ago",
		 _tx_count, _tx_drop,
		 (uint32_t)(_last_send == 0 ? 999999999u : (now > _last_send ? (uint32_t)(now - _last_send) : 0u)));

	PX4_INFO("mode %s (FS_UT_MTURN=%lli)",
		 _param_fs_ut_mturn.get() != 0 ? "Multi-turn" : "Single-turn",
		 (long long)_param_fs_ut_mturn.get());

	PX4_INFO("gain %.4f trim %.2f intv %lld ms tac %lld tdc %lld poll %llu Hz",
		 (double)_param_fs_ut_gain.get(),
		 (double)_param_fs_ut_trm.get(),
		 (long long)_param_fs_ut_intv.get(),
		 (long long)_param_fs_ut_tac.get(),
		 (long long)_param_fs_ut_tdc.get(),
		 (unsigned long long)_param_fs_ut_r_hz.get());

	/* 总线统计（始终显示）*/
	PX4_INFO("TX bps %.0f (~%.1f kbps) Hz %.1f  drop_dup %" PRIu32,
		 (double)_tx_bps, (double)(_tx_bps / 1000.0f), (double)_tx_hz, _ca_drop_dup);
	PX4_INFO("RX bps %.0f (~%.1f kbps)  util %.2f%%",
		 (double)_rx_bps, (double)(_rx_bps / 1000.0f), (double)_bus_util_pct);
	PX4_INFO("CA samples %.1f Hz   effective ctrl %.1f Hz (min(tx,ca))",
		 (double)_ca_hz, (double)_effective_ctrl_hz);

	if (_param_fs_ut_fb.get() != 0) {
		PX4_INFO("feedback ON  online=0x%02X  q_ok=%" PRIu32 "  q_err=%" PRIu32,
			 _online_flags, _query_ok_total, _query_err_total);
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
			 (unsigned long long)k_query_timeout_us);

	} else {
		PX4_INFO("feedback OFF (set FS_UT_FB=1 to enable angle readback)");
	}

	return 0;
}

extern "C" __EXPORT int fs_uart_servo_main(int argc, char *argv[])
{
	return FsUartServo::main(argc, argv);
}
