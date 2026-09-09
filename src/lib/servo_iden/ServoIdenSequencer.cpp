/****************************************************************************
 *
 * 自适应舵机扫频状态机实现。
 *
 ****************************************************************************/

#include "ServoIdenSequencer.hpp"

#include <math.h>

#include <mathlib/mathlib.h>
#include <px4_platform_common/defines.h>

namespace servo_iden
{

void ServoIdenSequencer::reset()
{
	_cfg = {};
	_phase = Phase::Idle;
	_phase_t0 = 0;
	_rep_idx = 0;
	_half_active = false;
	_vmax_dps = 0.f;
	_f1_eff_hz = 0.f;
	_fb_source = FbSource::None;
	_gyro_peak_dps = 0.f;
	_angle_slope_peak_dps = 0.f;
	_ang_n = 0;
	_prev_y = 0.f;
	_post_duration = false;
}

ServoIdenSequencer::Config ServoIdenSequencer::sanitize(const Config &in)
{
	Config c{};
	c.f0 = math::constrain(in.f0, 0.05f, 10.f);
	c.f1 = math::constrain(in.f1, c.f0 + 0.05f, 80.f);
	c.dur_s = math::constrain(in.dur_s, 10.f, 600.f);
	c.amp_deg = math::constrain(in.amp_deg, 1.f, 45.f);
	c.amin_deg = math::constrain(in.amin_deg, 0.2f, c.amp_deg);
	c.step_deg = math::constrain(in.step_deg, 2.f, 45.f);
	c.rep = math::constrain(in.rep, 1, 5);
	c.half_run = in.half_run;
	return c;
}

float ServoIdenSequencer::amplitude_at(float freq_hz, float a0, float amin, float vmax, uint8_t fb_source)
{
	if (fb_source == FbSource::None || vmax < 1.f || freq_hz <= 0.f) {
		return a0;
	}

	const float a_max = math::max(a0, 0.2f);
	const float a_min = math::min(amin, a_max);
	const float a_rate = k_rate_margin * vmax / (2.f * M_PI_F * freq_hz);
	return math::constrain(a_rate, a_min, a_max);
}

void ServoIdenSequencer::eval_log_chirp(float t, float f0, float f1, float duration,
					float &phi, float &f_inst)
{
	const float log_ratio = logf(f1 / f0);
	const float t_clamped = math::max(t, 0.f);
	f_inst = f0 * expf(t_clamped / duration * log_ratio);
	phi = 2.f * M_PI_F * f0 * duration / log_ratio * (expf(t_clamped / duration * log_ratio) - 1.f);
}

void ServoIdenSequencer::start_phase(Phase phase, hrt_abstime now)
{
	_phase = phase;
	_phase_t0 = now;
	_prev_y = 0.f;
	_post_duration = false;
}

float ServoIdenSequencer::current_f1_use() const
{
	if (_fb_source == FbSource::None || _vmax_dps < 1.f) {
		return _cfg.f1;
	}

	return math::min(_cfg.f1, math::max(_f1_eff_hz, _cfg.f0 + 0.05f));
}

void ServoIdenSequencer::finish_slew_test()
{
	if (_gyro_peak_dps > k_gyro_on_arm_dps) {
		_vmax_dps = _gyro_peak_dps;
		_fb_source = FbSource::Gyro;

	} else if (_angle_slope_peak_dps > k_angle_vmax_min_dps) {
		_vmax_dps = _angle_slope_peak_dps;
		_fb_source = FbSource::BusAngle;

	} else {
		_vmax_dps = 0.f;
		_fb_source = FbSource::None;
	}

	if (_fb_source == FbSource::None || _cfg.amin_deg <= 0.f) {
		_f1_eff_hz = _cfg.f1;

	} else {
		_f1_eff_hz = k_rate_margin * _vmax_dps / (2.f * M_PI_F * _cfg.amin_deg);
	}
}

void ServoIdenSequencer::start_next_lead_in(hrt_abstime now, bool half)
{
	_half_active = half;
	start_phase(Phase::LeadIn, now);
}

void ServoIdenSequencer::after_sweep(hrt_abstime now)
{
	if (!_half_active) {
		_rep_idx++;

		const bool more_full = _rep_idx < static_cast<uint8_t>(_cfg.rep);
		const bool do_half = _cfg.half_run;

		if (more_full || do_half) {
			start_phase(Phase::Pause, now);

		} else {
			start_phase(Phase::Done, now);
		}

	} else {
		start_phase(Phase::Done, now);
	}
}

void ServoIdenSequencer::collect_slew_feedback(hrt_abstime now, const Feedback &fb, float cmd_deg)
{
	const bool moving = fabsf(cmd_deg) > 1.f;

	if (!moving) {
		return;
	}

	if (fb.gyro_valid && PX4_ISFINITE(fb.gyro_dps)) {
		_gyro_peak_dps = math::max(_gyro_peak_dps, fabsf(fb.gyro_dps));
	}

	if (!fb.angle_valid || !PX4_ISFINITE(fb.angle_deg)) {
		return;
	}

	if (_ang_n > 0) {
		const hrt_abstime dt_us = now - _ang_t_hist[_ang_n - 1];

		if (dt_us < 1000) {
			return;
		}
	}

	if (_ang_n < 3) {
		_ang_hist[_ang_n] = fb.angle_deg;
		_ang_t_hist[_ang_n] = now;
		_ang_n++;

	} else {
		_ang_hist[0] = _ang_hist[1];
		_ang_t_hist[0] = _ang_t_hist[1];
		_ang_hist[1] = _ang_hist[2];
		_ang_t_hist[1] = _ang_t_hist[2];
		_ang_hist[2] = fb.angle_deg;
		_ang_t_hist[2] = now;
	}

	if (_ang_n == 3) {
		const float dt_s = (_ang_t_hist[2] - _ang_t_hist[0]) * 1e-6f;

		if (dt_s > 0.002f) {
			const float slope = fabsf((_ang_hist[2] - _ang_hist[0]) / dt_s);
			_angle_slope_peak_dps = math::max(_angle_slope_peak_dps, slope);
		}
	}
}

ServoIdenSequencer::Output ServoIdenSequencer::make_output(float cmd_deg, float sweep_t,
		float inst_freq, float amp) const
{
	Output o{};
	o.cmd_deg = cmd_deg;
	o.phase = static_cast<uint8_t>(_phase);
	o.rep_idx = _half_active ? static_cast<uint8_t>(_cfg.rep) : _rep_idx;
	o.sweep_t = sweep_t;
	o.inst_freq_hz = inst_freq;
	o.amp_deg = amp;
	o.vmax_dps = _vmax_dps;
	o.f1_eff_hz = _f1_eff_hz;
	o.fb_source = _fb_source;
	o.done = (_phase == Phase::Done);
	return o;
}

ServoIdenSequencer::Output ServoIdenSequencer::update(hrt_abstime now, const Config &config,
		const Feedback &fb)
{
	if (_phase == Phase::Idle) {
		_cfg = sanitize(config);
		_rep_idx = 0;
		_half_active = false;
		_vmax_dps = 0.f;
		_f1_eff_hz = _cfg.f1;
		_fb_source = FbSource::None;
		_gyro_peak_dps = 0.f;
		_angle_slope_peak_dps = 0.f;
		_ang_n = 0;
		start_phase(Phase::Settle, now);
	}

	if (_phase == Phase::Done) {
		return make_output(0.f, 0.f, 0.f, 0.f);
	}

	if (_phase_t0 == 0) {
		_phase_t0 = now;
	}

	const float t = (now - _phase_t0) * 1e-6f;

	switch (_phase) {
	case Phase::Settle:
		if (t >= k_settle_s) {
			_ang_n = 0;
			_gyro_peak_dps = 0.f;
			_angle_slope_peak_dps = 0.f;
			start_phase(Phase::SlewTest, now);
			return make_output(0.f, 0.f, 0.f, 0.f);
		}

		return make_output(0.f, 0.f, 0.f, 0.f);

	case Phase::SlewTest: {
			const float total = k_slew_seg_s * static_cast<float>(k_slew_n_seg);
			int seg = static_cast<int>(t / k_slew_seg_s);

			if (seg < 0) {
				seg = 0;
			}

			float cmd = 0.f;

			if (seg == 1) {
				cmd = _cfg.step_deg;

			} else if (seg == 2) {
				cmd = -_cfg.step_deg;
			}

			collect_slew_feedback(now, fb, cmd);

			if (t >= total) {
				finish_slew_test();
				start_next_lead_in(now, false);
				return make_output(0.f, 0.f, 0.f, 0.f);
			}

			return make_output(cmd, 0.f, 0.f, fabsf(cmd));
		}

	case Phase::LeadIn: {
			const float f0 = _cfg.f0;
			const float lead_s = static_cast<float>(k_lead_periods) / f0;
			const float a0 = current_a0();
			const float amp = amplitude_at(f0, a0, _cfg.amin_deg, _vmax_dps, _fb_source);
			const float cmd = amp * sinf(2.f * M_PI_F * f0 * t);

			if (t >= lead_s) {
				start_phase(_half_active ? Phase::SweepHalf : Phase::Sweep, now);
				return make_output(0.f, 0.f, f0, amp);
			}

			return make_output(cmd, t, f0, amp);
		}

	case Phase::Sweep:
	case Phase::SweepHalf: {
			const float f0 = _cfg.f0;
			const float f1 = current_f1_use();
			const float dur = _cfg.dur_s;
			const float a0 = current_a0();
			float phi = 0.f;
			float f_inst = f0;
			eval_log_chirp(t, f0, f1, dur, phi, f_inst);
			const float amp = amplitude_at(f_inst, a0, _cfg.amin_deg, _vmax_dps, _fb_source);
			const float y = amp * sinf(phi);

			if (t >= dur) {
				const bool crossed = (_prev_y * y <= 0.f) || (fabsf(y) < 1e-3f);

				if (_post_duration && crossed) {
					after_sweep(now);
					return make_output(0.f, t, f_inst, 0.f);
				}

				_post_duration = true;
			}

			_prev_y = y;
			return make_output(y, t, f_inst, amp);
		}

	case Phase::Pause:
		if (t >= k_pause_s) {
			const bool more_full = _rep_idx < static_cast<uint8_t>(_cfg.rep);
			start_next_lead_in(now, !more_full);
			return make_output(0.f, 0.f, 0.f, 0.f);
		}

		return make_output(0.f, 0.f, 0.f, 0.f);

	case Phase::Idle:
	case Phase::Done:
	default:
		break;
	}

	return make_output(0.f, 0.f, 0.f, 0.f);
}

} // namespace servo_iden
