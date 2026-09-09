/****************************************************************************
 *
 * FashionStar UART bus servo output (control_allocator actuator_servos → bus).
 *
 ****************************************************************************/

#pragma once

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <string.h>

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/manual_control_switches.h>
#include <uORB/topics/actuator_servos.h>
#include <uORB/topics/Fs_data.h>
#include <uORB/topics/Servo_iden_ctrl.h>
#include <uORB/topics/Servo_iden_data.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_attitude.h>

#include <drivers/drv_hrt.h>
#include <math.h>

#include <lib/mathlib/mathlib.h>
#include <lib/servo_iden/ServoIdenSequencer.hpp>

#include "fs_protocol.hpp"

using namespace time_literals;

class FsUartServo : public ModuleBase<FsUartServo>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	/** @param device_path Resolved UART path (non-empty); used for serial_port_to_wq. */
	FsUartServo(const char *device_path, int baud_cli);
	~FsUartServo() override;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	bool init();
	int print_status() override;

private:
	void Run() override;

	void update_params(const bool force);
	uint32_t schedule_interval_us() const;
	void apply_service_schedule(); ///< 辨识启停时切换 4 kHz / 普通轮询节拍
	bool uart_is_1mbaud() const;
	bool feedback_enabled() const; ///< FS_UT_FB=1，或辨识扫频期间自动回传
	uint32_t iden_ctrl_hz() const; ///< 总线辨识发送/回传频率（IDEN_SV_HZ）
	hrt_abstime iden_ctrl_interval_us() const;
	uint32_t feedback_target_hz_per_ch() const;
	hrt_abstime query_timeout_us() const;
	void refill_query_budget(hrt_abstime now);

	void refresh_servo_kill_state();

	bool open_uart();
	void close_uart();

	void send_latest_frame(const actuator_servos_s &sv);
	void send_zero_sync_frame(); ///< UART 就绪后主动下发 0°，不依赖 actuator_servos 订阅
	/** 按最终角度组同步帧。skip_sample_dedup 时不按 CA timestamp 去重（辨识 5 ms 节拍）。 */
	void send_sync_deg(const float angle_deg[actuator_servos_s::NUM_CONTROLS],
			   uint64_t timestamp_sample, bool skip_sample_dedup);
	void send_iden_frame(uint8_t idx, float cmd_deg);

	void advance_query_id();
	servo_iden::ServoIdenSequencer::Config make_iden_config() const;
	void fill_iden_feedback(servo_iden::ServoIdenSequencer::Feedback &fb);
	void publish_servo_iden_data(const servo_iden::ServoIdenSequencer::Output &out,
				     uint8_t servo_idx, hrt_abstime tx_time);

	/* --- 角度回传（FS_UT_FB=1，或辨识扫频期间自动开启）--- */
	void poll_rx();                     ///< 排空 UART RX，解析并更新 fb_*
	bool maybe_send_query();            ///< 窗口允许时发下一帧查询
	void on_query_timeout();            ///< 当前 ID 掉线，轮询前进
	void disable_feedback();            ///< 回传关闭（参数关且未在辨识）
	void enable_feedback();             ///< 回传开启

	/* --- 统计 / Fs_data --- */
	void update_stats_window();         ///< 1s window diff → snapshot
	void publish_fs_data();             ///< write Fs_data topic (up to 200 Hz)

	char _device_path[64] {};
	int _baud_cli{-1};
	int _uart_fd{-1};
	int _effective_baud{-1};
	int _line_baud{-1}; ///< NuttX UART rate after tcsetattr (0 = unknown)
	bool _single_wire_active{false};

	float _last_deg[actuator_servos_s::NUM_CONTROLS] {};
	float _last_cmd_deg[actuator_servos_s::NUM_CONTROLS] {}; ///< last sync frame deg (after gain/trim/kill)
	uint16_t _last_tx_len{0};
	actuator_servos_s _last_srv{};
	bool _last_srv_valid{false}; ///< true after at least one actuator_servos sample (not on UART open)

	uint64_t _last_sent_sample{0}; ///< last sent actuator_servos.timestamp_sample
	bool _sent_once{false};
	bool _send_pending{false};     ///< rate-limited; retry on next Run

	uint32_t _tx_count{0};
	uint32_t _tx_drop{0};
	hrt_abstime _last_send{0};

	/* 同步帧发送完成后的"安静期"，让总线给舵机处理时间，避免与查询冲突 */
	hrt_abstime _post_tx_until{0};

	/* --- 回传状态机 --- */
	bool _fb_enabled_prev{false};
	uint8_t _query_id{0};            ///< 0..7 当前正在等待响应的 ID
	hrt_abstime _query_tx_us{0};     ///< 查询帧发出时间；0 表示当前空闲
	hrt_abstime _next_query_budget_us{0}; ///< 下一个 5 ms 查询配额窗口
	uint32_t _query_credit{0};       ///< 分数配额，单位为 query Hz / 200 Hz control window
	uint8_t _query_budget{0};        ///< 当前 5 ms 窗口尚可发送的查询数
	uint32_t _service_interval_us{0}; ///< ScheduledWorkItem 实际服务周期
	float _fb_angle_deg[actuator_servos_s::NUM_CONTROLS] {}; ///< 最近一次成功回传角度
	uint8_t _online_flags{0};
	fs_uart_servo::ResponseParser _rx_parser{};

	/* --- 1s 窗口累加 --- */
	hrt_abstime _stats_t0{0};
	uint32_t _tx_bytes_win{0}, _rx_bytes_win{0};
	uint32_t _tx_frames_win{0}, _ca_samples_win{0};
	uint32_t _query_ok_win{0};
	uint32_t _query_ok_per_ch_win[actuator_servos_s::NUM_CONTROLS] {};

	/* --- 1s 窗口结束后的显示快照（写入 Fs_data / status）--- */
	float _tx_bps{0.f}, _rx_bps{0.f}, _bus_util_pct{0.f};
	float _tx_hz{0.f}, _ca_hz{0.f}, _effective_ctrl_hz{0.f};
	float _query_hz_per_ch[actuator_servos_s::NUM_CONTROLS] {};

	/* --- 累计计数（不重置）--- */
	uint32_t _ca_drop_dup{0};
	uint32_t _query_ok_total{0};
	uint32_t _query_err_total{0};
	uint32_t _rx_frame_total{0};       ///< checksum-valid angle response frames, including unmatched ones
	uint32_t _rx_match_miss_total{0};  ///< valid response whose cmd/id did not match the outstanding query
	uint32_t _rx_checksum_err_total{0};
	uint8_t _rx_seen_id_flags{0};      ///< response IDs 0..7 observed on the wire, regardless of query match
	uint8_t _last_rx_cmd{0};
	uint8_t _last_rx_id{UINT8_MAX};
	uint8_t _last_rx_size{0};

	/* --- RTT --- */
	float _rtt_ema_us{0.f};
	float _rtt_max_us{0.f};

	/* Fs_data 发布节拍 */
	hrt_abstime _last_fs_data_pub{0};

	uORB::Subscription _actuator_servos_sub{ORB_ID(actuator_servos)};
	uORB::Subscription _actuator_armed_sub{ORB_ID(actuator_armed)};
	uORB::Subscription _manual_control_switches_sub{ORB_ID(manual_control_switches)};
	uORB::Subscription _servo_iden_ctrl_sub{ORB_ID(Servo_iden_ctrl)};
	uORB::Subscription _vehicle_angular_velocity_sub{ORB_ID(vehicle_angular_velocity)};
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	uORB::Publication<Fs_data_s> _fs_data_pub{ORB_ID(Fs_data)};
	uORB::Publication<Servo_iden_data_s> _servo_iden_data_pub{ORB_ID(Servo_iden_data)};

	bool _servo_kill_zero{false};
	bool _servo_kill_zero_prev{false};

	servo_iden::ServoIdenSequencer _iden_seq;
	servo_iden::ServoIdenSequencer::Output _iden_last_out{};
	bool _iden_completed_this_arm{false};
	bool _iden_was_armed{false};
	bool _iden_pin_query{false};
	bool _iden_fb_active{false}; ///< 扫频进行中：自动回传，频率与 IDEN_SV_HZ 相同
	uint8_t _iden_servo_idx{0};
	float _gyro_dps[3] {};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::FS_UT_GAIN>) _param_fs_ut_gain,
		(ParamFloat<px4::params::FS_UT_TRM>) _param_fs_ut_trm,
		(ParamInt<px4::params::FS_UT_BAUD>) _param_fs_ut_baud,
		(ParamInt<px4::params::FS_UT_INTV>) _param_fs_ut_intv,
		(ParamInt<px4::params::FS_UT_TAC>) _param_fs_ut_tac,
		(ParamInt<px4::params::FS_UT_TDC>) _param_fs_ut_tdc,
		(ParamInt<px4::params::FS_UT_R_HZ>) _param_fs_ut_r_hz,
		(ParamInt<px4::params::FS_UT_MTURN>) _param_fs_ut_mturn,
		(ParamInt<px4::params::FS_UT_FB>) _param_fs_ut_fb,
		(ParamInt<px4::params::FS_UT_1WIRE>) _param_fs_ut_1wire,
		(ParamFloat<px4::params::IDEN_CH_F0>) _param_iden_ch_f0,
		(ParamFloat<px4::params::IDEN_CH_F1>) _param_iden_ch_f1,
		(ParamFloat<px4::params::IDEN_CH_DUR>) _param_iden_ch_dur,
		(ParamFloat<px4::params::IDEN_CH_AMP>) _param_iden_ch_amp,
		(ParamFloat<px4::params::IDEN_CH_AMIN>) _param_iden_ch_amin,
		(ParamFloat<px4::params::IDEN_CH_STEP>) _param_iden_ch_step,
		(ParamInt<px4::params::IDEN_CH_REP>) _param_iden_ch_rep,
		(ParamInt<px4::params::IDEN_CH_HALF>) _param_iden_ch_half,
		(ParamInt<px4::params::IDEN_SV_HZ>) _param_iden_sv_hz
	)
};
