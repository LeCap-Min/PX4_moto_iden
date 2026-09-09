/****************************************************************************
 *
 * 自适应舵机扫频状态机：静置（3 s）→ 速率自测 → 引导 → 对数扫频（可重复）→ 半幅检验。
 *
 ****************************************************************************/

#pragma once

#include <stdint.h>

#include <drivers/drv_hrt.h>

namespace servo_iden
{

class ServoIdenSequencer
{
public:
	/** 写入 Servo_iden_data.phase */
	enum Phase : uint8_t {
		Idle = 0,
		Settle = 1,
		SlewTest = 2,
		LeadIn = 3,
		Sweep = 4,
		Pause = 5,
		SweepHalf = 6,
		Done = 7
	};

	/** 写入 Servo_iden_data.fb_source */
	enum FbSource : uint8_t {
		None = 0,
		Gyro = 1,
		BusAngle = 2
	};

	struct Config {
		float f0{0.2f};       ///< 起始频率 Hz
		float f1{30.f};       ///< 扫频上限 Hz
		float dur_s{90.f};    ///< 单次扫频时长 s
		float amp_deg{8.f};   ///< 低频幅值 A0 度
		float amin_deg{1.5f}; ///< 最小幅值 度
		float step_deg{15.f}; ///< 速率自测阶跃幅值 度
		int   rep{3};         ///< 全幅扫频重复次数
		bool  half_run{true}; ///< 是否追加半幅扫频
	};

	struct Feedback {
		float angle_deg{0.f};
		bool  angle_valid{false};
		float gyro_dps{0.f};  ///< 三轴角速度模长，度/秒
		bool  gyro_valid{false};
	};

	struct Output {
		float   cmd_deg{0.f};
		uint8_t phase{Phase::Idle};
		uint8_t rep_idx{0};
		float   sweep_t{0.f};
		float   inst_freq_hz{0.f};
		float   amp_deg{0.f};
		float   vmax_dps{0.f};
		float   f1_eff_hz{0.f};
		uint8_t fb_source{FbSource::None};
		bool    done{false};
	};

	void reset();

	/** 按 now 推进状态机；波形与时间绑定，漏调用只会少发一帧，相位仍连续。 */
	Output update(hrt_abstime now, const Config &config, const Feedback &fb);

private:
	static constexpr float k_rate_margin = 0.6f;       ///< 速率裕度，不暴露为参数
	static constexpr float k_gyro_on_arm_dps = 30.f;   ///< 超过此峰值视为 IMU 装在舵臂上
	static constexpr float k_angle_vmax_min_dps = 10.f;
	static constexpr float k_settle_s = 3.f; ///< 静置：先钉查询、等 RTT 稳定，再做速率自测
	static constexpr float k_slew_seg_s = 1.f;
	static constexpr int   k_slew_n_seg = 4;
	static constexpr float k_pause_s = 2.f;
	static constexpr int   k_lead_periods = 2;

	static Config sanitize(const Config &in);
	static float amplitude_at(float freq_hz, float a0, float amin, float vmax, uint8_t fb_source);
	static void eval_log_chirp(float t, float f0, float f1, float duration,
				   float &phi, float &f_inst);

	void start_phase(Phase phase, hrt_abstime now);
	void finish_slew_test();
	void start_next_lead_in(hrt_abstime now, bool half);
	void after_sweep(hrt_abstime now);
	void collect_slew_feedback(hrt_abstime now, const Feedback &fb, float cmd_deg);
	Output make_output(float cmd_deg, float sweep_t, float inst_freq, float amp) const;
	float current_a0() const { return _half_active ? (0.5f * _cfg.amp_deg) : _cfg.amp_deg; }
	float current_f1_use() const;

	Config _cfg{};
	Phase _phase{Phase::Idle};
	hrt_abstime _phase_t0{0};

	uint8_t _rep_idx{0};
	bool _half_active{false};

	float _vmax_dps{0.f};
	float _f1_eff_hz{0.f};
	uint8_t _fb_source{FbSource::None};

	float _gyro_peak_dps{0.f};
	float _angle_slope_peak_dps{0.f};
	float _ang_hist[3] {};
	hrt_abstime _ang_t_hist[3] {};
	uint8_t _ang_n{0};

	float _prev_y{0.f};
	bool _post_duration{false};
};

} // namespace servo_iden
