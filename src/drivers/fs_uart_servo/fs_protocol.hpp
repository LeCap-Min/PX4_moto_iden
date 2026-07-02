/****************************************************************************
 *
 * Minimal FashionStar bus protocol framer — sync command MODE_SET_SERVO_ANGLE_BY_INTERVAL.
 * Mirrors fsuservo::FSUS_Protocol::sendSyncCommand (... MODE_SET_SERVO_ANGLE_BY_INTERVAL).
 *
 ****************************************************************************/

#pragma once

#include <stdint.h>
#include <unistd.h>

namespace fs_uart_servo
{

static constexpr uint16_t k_pack_request_header = 0x4c12;
static constexpr uint16_t k_pack_response_header = 0x1c05;
static constexpr uint8_t k_cmd_set_servo_sync = 25;
static constexpr uint8_t k_cmd_query_angle = 10;
static constexpr uint8_t k_cmd_query_angle_mturn = 16;

struct SyncServoParam {
	uint8_t id {};          ///< bus ID 0 … 254
	float angle_deg {};     ///< commanded angle (degrees), ±180 clamped inside builder
	uint16_t interval_ms {};
	uint16_t t_acc_ms {};
	uint16_t t_dec_ms {};
	uint16_t power_mw {};    ///< 0 = default drive power handling
};

/**
 * Encode one complete wire frame including header / cmdId / checksum.
 * Wire layout matches FashionStar cpp SDK sendPack()+sendSyncCommand (interval mode).
 *
 * @return frame length written to out_buf, or 0 on error (capacity / count).
 */
inline uint16_t build_sync_angle_by_interval_frame(const SyncServoParam *servos, uint8_t count,
		uint8_t *out_buf, uint16_t out_cap)
{
	if (servos == nullptr || out_buf == nullptr || count == 0) {
		return 0;
	}

	const unsigned content_bytes = static_cast<unsigned>(3) + static_cast<unsigned>(count) * 11U;

	const unsigned single_byte_len_field = (content_bytes < 255U) ? 1U : 3U;
	const unsigned wire_len =
		2U + 1U + single_byte_len_field + content_bytes + 1U /* checksum */;

	if (wire_len > out_cap) {
		return 0;
	}

	uint16_t idx = 0;
	out_buf[idx++] = static_cast<uint8_t>(k_pack_request_header & 0xFFU);
	out_buf[idx++] = static_cast<uint8_t>(k_pack_request_header >> 8);
	out_buf[idx++] = k_cmd_set_servo_sync;

	if (content_bytes < 255U) {
		out_buf[idx++] = static_cast<uint8_t>(content_bytes);

	} else {
		out_buf[idx++] = 0xFF;
		out_buf[idx++] = static_cast<uint8_t>(content_bytes & 0xFFU);
		out_buf[idx++] = static_cast<uint8_t>(content_bytes >> 8);
	}

	// MODE_SET_SERVO_ANGLE_BY_INTERVAL (subcmd layout from SDK): 11 bytes per servo, sub cmd 11
	out_buf[idx++] = 11;
	out_buf[idx++] = 11;
	out_buf[idx++] = count;

	for (uint8_t si = 0; si < count; si++) {
		float ang = servos[si].angle_deg;

		if (ang > 180.0f) {
			ang = 180.0f;

		} else if (ang < -180.0f) {
			ang = -180.0f;
		}

		const int16_t angle_int10 = static_cast<int16_t>(10.f * ang);
		out_buf[idx++] = servos[si].id;
		out_buf[idx++] = static_cast<uint8_t>(angle_int10 & 0xFF);
		out_buf[idx++] = static_cast<uint8_t>((angle_int10 >> 8) & 0xFF);

		out_buf[idx++] = static_cast<uint8_t>(servos[si].interval_ms & 0xFFU);
		out_buf[idx++] = static_cast<uint8_t>((servos[si].interval_ms >> 8) & 0xFFU);

		out_buf[idx++] = static_cast<uint8_t>(servos[si].t_acc_ms & 0xFFU);
		out_buf[idx++] = static_cast<uint8_t>((servos[si].t_acc_ms >> 8) & 0xFFU);

		out_buf[idx++] = static_cast<uint8_t>(servos[si].t_dec_ms & 0xFFU);
		out_buf[idx++] = static_cast<uint8_t>((servos[si].t_dec_ms >> 8) & 0xFFU);

		out_buf[idx++] = static_cast<uint8_t>(servos[si].power_mw & 0xFFU);
		out_buf[idx++] = static_cast<uint8_t>((servos[si].power_mw >> 8) & 0xFFU);
	}

	// FashionStar wire checksum: byte sum of header..last content byte modulo 256 (see SDK calcPackChecksum).
	uint16_t sum = 0;

	for (uint16_t k = 0; k < idx; k++) {
		sum += out_buf[k];
	}

	out_buf[idx++] = static_cast<uint8_t>(sum % 256U);
	return idx;
}

/**
 * MODE_SET_SERVO_ANGLE_MTURN_BY_INTERVAL — 15 bytes per servo (subcmd 14/15).
 * Angle int32 ×10 (LE), interval uint32 (LE), t_acc/t_dec/power same widths as single-turn.
 */
inline uint16_t build_sync_angle_mturn_by_interval_frame(const SyncServoParam *servos, uint8_t count,
		uint8_t *out_buf, uint16_t out_cap)
{
	if (servos == nullptr || out_buf == nullptr || count == 0) {
		return 0;
	}

	const unsigned content_bytes = static_cast<unsigned>(3) + static_cast<unsigned>(count) * 15U;

	const unsigned single_byte_len_field = (content_bytes < 255U) ? 1U : 3U;
	const unsigned wire_len =
		2U + 1U + single_byte_len_field + content_bytes + 1U /* checksum */;

	if (wire_len > out_cap) {
		return 0;
	}

	uint16_t idx = 0;
	out_buf[idx++] = static_cast<uint8_t>(k_pack_request_header & 0xFFU);
	out_buf[idx++] = static_cast<uint8_t>(k_pack_request_header >> 8);
	out_buf[idx++] = k_cmd_set_servo_sync;

	if (content_bytes < 255U) {
		out_buf[idx++] = static_cast<uint8_t>(content_bytes);

	} else {
		out_buf[idx++] = 0xFF;
		out_buf[idx++] = static_cast<uint8_t>(content_bytes & 0xFFU);
		out_buf[idx++] = static_cast<uint8_t>(content_bytes >> 8);
	}

	out_buf[idx++] = 14;
	out_buf[idx++] = 15;
	out_buf[idx++] = count;

	for (uint8_t si = 0; si < count; si++) {
		float ang = servos[si].angle_deg;

		if (ang > 368640.0f) {
			ang = 368640.0f;

		} else if (ang < -368640.0f) {
			ang = -368640.0f;
		}

		const int32_t angle_int10 = static_cast<int32_t>(10.f * ang);
		const uint32_t interval_multi = static_cast<uint32_t>(servos[si].interval_ms);

		out_buf[idx++] = servos[si].id;
		out_buf[idx++] = static_cast<uint8_t>(angle_int10 & 0xFF);
		out_buf[idx++] = static_cast<uint8_t>((angle_int10 >> 8) & 0xFF);
		out_buf[idx++] = static_cast<uint8_t>((angle_int10 >> 16) & 0xFF);
		out_buf[idx++] = static_cast<uint8_t>((angle_int10 >> 24) & 0xFF);

		out_buf[idx++] = static_cast<uint8_t>(interval_multi & 0xFFU);
		out_buf[idx++] = static_cast<uint8_t>((interval_multi >> 8) & 0xFFU);
		out_buf[idx++] = static_cast<uint8_t>((interval_multi >> 16) & 0xFFU);
		out_buf[idx++] = static_cast<uint8_t>((interval_multi >> 24) & 0xFFU);

		out_buf[idx++] = static_cast<uint8_t>(servos[si].t_acc_ms & 0xFFU);
		out_buf[idx++] = static_cast<uint8_t>((servos[si].t_acc_ms >> 8) & 0xFFU);

		out_buf[idx++] = static_cast<uint8_t>(servos[si].t_dec_ms & 0xFFU);
		out_buf[idx++] = static_cast<uint8_t>((servos[si].t_dec_ms >> 8) & 0xFFU);

		out_buf[idx++] = static_cast<uint8_t>(servos[si].power_mw & 0xFFU);
		out_buf[idx++] = static_cast<uint8_t>((servos[si].power_mw >> 8) & 0xFFU);
	}

	uint16_t sum = 0;

	for (uint16_t k = 0; k < idx; k++) {
		sum += out_buf[k];
	}

	out_buf[idx++] = static_cast<uint8_t>(sum % 256U);
	return idx;
}

/**
 * Build single-turn QUERY_ANGLE frame (cmd 10): request angle of one servo.
 * Wire layout: header(2) + cmdId(1) + content_size(1)=1 + id(1) + checksum(1) = 6 bytes.
 *
 * @return frame length (6) or 0 if out_cap insufficient.
 */
inline uint16_t build_query_angle_frame(uint8_t id, uint8_t *out_buf, uint16_t out_cap)
{
	if (out_buf == nullptr || out_cap < 6) {
		return 0;
	}

	uint16_t idx = 0;
	out_buf[idx++] = static_cast<uint8_t>(k_pack_request_header & 0xFFU);
	out_buf[idx++] = static_cast<uint8_t>(k_pack_request_header >> 8);
	out_buf[idx++] = k_cmd_query_angle;
	out_buf[idx++] = 1;
	out_buf[idx++] = id;

	uint16_t sum = 0;

	for (uint16_t k = 0; k < idx; k++) {
		sum += out_buf[k];
	}

	out_buf[idx++] = static_cast<uint8_t>(sum % 256U);
	return idx;
}

/**
 * Build multi-turn QUERY_ANGLE_MTURN frame (cmd 16): same payload as single-turn but
 * response carries int32 angle.
 *
 * @return frame length (6) or 0 if out_cap insufficient.
 */
inline uint16_t build_query_angle_mturn_frame(uint8_t id, uint8_t *out_buf, uint16_t out_cap)
{
	if (out_buf == nullptr || out_cap < 6) {
		return 0;
	}

	uint16_t idx = 0;
	out_buf[idx++] = static_cast<uint8_t>(k_pack_request_header & 0xFFU);
	out_buf[idx++] = static_cast<uint8_t>(k_pack_request_header >> 8);
	out_buf[idx++] = k_cmd_query_angle_mturn;
	out_buf[idx++] = 1;
	out_buf[idx++] = id;

	uint16_t sum = 0;

	for (uint16_t k = 0; k < idx; k++) {
		sum += out_buf[k];
	}

	out_buf[idx++] = static_cast<uint8_t>(sum % 256U);
	return idx;
}

/**
 * Non-blocking response frame parser. Feed one byte at a time; emits a parsed frame
 * via the callback when complete. Mirrors SDK recvPack() state machine.
 *
 * Response wire layout: header(2)=0x1c05 + cmdId(1) + content_size(1) + content[N] + checksum(1).
 */
class ResponseParser
{
public:
	struct Frame {
		uint8_t cmd_id;
		uint8_t content_size;
		uint8_t content[64];
		uint16_t total_len;     ///< bytes on wire including header+checksum
	};

	enum class Result : uint8_t {
		Need_more,
		Frame_ok,
		Header_err,
		Cmd_err,
		Size_err,
		Checksum_err,
	};

	void reset()
	{
		_state = State::Wait_h0;
		_idx = 0;
		_sum = 0;
		_content_size = 0;
		_cmd_id = 0;
	}

	/** Feed one byte; returns Need_more until a full frame is parsed/rejected. */
	Result feed(uint8_t b, Frame &out)
	{
		static constexpr uint8_t h0 = static_cast<uint8_t>(k_pack_response_header & 0xFFU);
		static constexpr uint8_t h1 = static_cast<uint8_t>(k_pack_response_header >> 8);

		switch (_state) {
		case State::Wait_h0:
			/* Only the response header (0x1c05) starts a frame. Bytes that don't match —
			 * including echoes of our own request frames (header 0x4c12) — are dropped here. */
			if (b == h0) {
				_sum = b;
				_state = State::Wait_h1;
			}

			return Result::Need_more;

		case State::Wait_h1:
			if (b == h1) {
				_sum += b;
				_state = State::Wait_cmd;
				return Result::Need_more;
			}

			/* Re-sync without losing a possible new header start: if this byte is itself h0,
			 * stay primed on it instead of dropping it. */
			reset();

			if (b == h0) {
				_sum = b;
				_state = State::Wait_h1;
			}

			return Result::Header_err;

		case State::Wait_cmd:
			_cmd_id = b;
			_sum += b;

			if (_cmd_id != k_cmd_query_angle && _cmd_id != k_cmd_query_angle_mturn) {
				/* Unexpected cmd: drop, but re-prime if this byte could be a header start. */
				reset();

				if (b == h0) {
					_sum = b;
					_state = State::Wait_h1;
				}

				return Result::Cmd_err;
			}

			_state = State::Wait_size;
			return Result::Need_more;

		case State::Wait_size:
			_content_size = b;
			_sum += b;

			if (_content_size == 0 || _content_size > sizeof(out.content)) {
				reset();

				if (b == h0) {
					_sum = b;
					_state = State::Wait_h1;
				}

				return Result::Size_err;
			}

			_idx = 0;
			_state = State::Wait_content;
			return Result::Need_more;

		case State::Wait_content:
			out.content[_idx++] = b;
			_sum += b;

			if (_idx >= _content_size) {
				_state = State::Wait_checksum;
			}

			return Result::Need_more;

		case State::Wait_checksum: {
				const uint8_t expected = static_cast<uint8_t>(_sum % 256U);
				out.cmd_id = _cmd_id;
				out.content_size = _content_size;
				out.total_len = 5 + _content_size; /* header2 + cmd1 + size1 + content + chk1 */
				const bool ok = (b == expected);
				reset();
				return ok ? Result::Frame_ok : Result::Checksum_err;
			}
		}

		reset();
		return Result::Need_more;
	}

private:
	enum class State : uint8_t {
		Wait_h0,
		Wait_h1,
		Wait_cmd,
		Wait_size,
		Wait_content,
		Wait_checksum,
	};

	State _state{State::Wait_h0};
	uint8_t _idx{0};
	uint16_t _sum{0};
	uint8_t _content_size{0};
	uint8_t _cmd_id{0};
};

/** Drain pending RX bytes on the UART (call before/after FB toggles to avoid stale frames). */
inline void flush_rx(int fd)
{
	if (fd < 0) {
		return;
	}

	uint8_t scratch[64];

	while (true) {
		const ssize_t n = ::read(fd, scratch, sizeof(scratch));

		if (n <= 0) {
			break;
		}
	}
}

} // namespace fs_uart_servo
