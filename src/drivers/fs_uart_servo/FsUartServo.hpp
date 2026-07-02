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
#include <uORB/topics/parameter_update.h>

#include <drivers/drv_hrt.h>
#include <math.h>

#include <lib/mathlib/mathlib.h>

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

	void refresh_servo_kill_state();

	bool open_uart();
	void close_uart();

	void send_latest_frame(const actuator_servos_s &sv);
	void send_zero_sync_frame(); ///< UART 就绪后主动下发 0°，不依赖 actuator_servos 订阅

	/* --- 角度回传（FS_UT_FB=1）--- */
	void poll_rx();                     ///< drain UART RX, feed parser, update fb_*
	void maybe_send_query();            ///< issue next query if window permits
	void on_query_timeout();            ///< mark current id offline, advance ring
	void disable_feedback();            ///< called when FS_UT_FB transitions 1->0
	void enable_feedback();             ///< called when FS_UT_FB transitions 0->1

	/* --- 统计 / Fs_data --- */
	void update_stats_window();         ///< 1s window diff → snapshot
	void publish_fs_data();             ///< write Fs_data topic (~10 Hz)

	char _device_path[64] {};
	int _baud_cli{-1};
	int _uart_fd{-1};
	int _effective_baud{-1};
	int _line_baud{-1}; ///< NuttX UART rate after tcsetattr (0 = unknown)

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

	/* --- RTT --- */
	float _rtt_ema_us{0.f};
	float _rtt_max_us{0.f};

	/* Fs_data 发布节拍 */
	hrt_abstime _last_fs_data_pub{0};

	uORB::Subscription _actuator_servos_sub{ORB_ID(actuator_servos)};
	uORB::Subscription _actuator_armed_sub{ORB_ID(actuator_armed)};
	uORB::Subscription _manual_control_switches_sub{ORB_ID(manual_control_switches)};
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	uORB::Publication<Fs_data_s> _fs_data_pub{ORB_ID(Fs_data)};

	bool _servo_kill_zero{false};
	bool _servo_kill_zero_prev{false};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::FS_UT_GAIN>) _param_fs_ut_gain,
		(ParamFloat<px4::params::FS_UT_TRM>) _param_fs_ut_trm,
		(ParamInt<px4::params::FS_UT_BAUD>) _param_fs_ut_baud,
		(ParamInt<px4::params::FS_UT_INTV>) _param_fs_ut_intv,
		(ParamInt<px4::params::FS_UT_TAC>) _param_fs_ut_tac,
		(ParamInt<px4::params::FS_UT_TDC>) _param_fs_ut_tdc,
		(ParamInt<px4::params::FS_UT_R_HZ>) _param_fs_ut_r_hz,
		(ParamInt<px4::params::FS_UT_MTURN>) _param_fs_ut_mturn,
		(ParamInt<px4::params::FS_UT_FB>) _param_fs_ut_fb
	)
};
