uint64 timestamp					# time since system start (microseconds)
uint64 timestamp_sample				# 关联的 actuator_servos.timestamp_sample

uint8 NUM_SERVOS = 8

# --- 命令侧（始终有效，不依赖 FS_UT_FB）---
float32[8] cmd_deg					# 本次下发到总线的命令角（度）
uint8 mturn_mode					# 0=单圈 1=多圈（= FS_UT_MTURN）
uint8 kill_zero						# 1 = 当前处于 kill 强制 0°
int32 line_baud						# cfgetspeed 实际值
uint16 last_tx_len					# 最近一次同步帧字节数

# --- 总线统计（1s 窗口差分；始终有效）---
float32 tx_bps						# 含 start/stop 位
float32 rx_bps
float32 bus_util_pct				# (tx_bps+rx_bps)/line_baud*100
float32 tx_frame_hz					# 实际下发 Hz
float32 ca_sample_hz				# actuator_servos.timestamp_sample 变化 Hz
float32 effective_ctrl_hz			# = min(tx_frame_hz, ca_sample_hz)，链路真实瓶颈速率（一个数总览）
uint32 tx_drop						# 累计 write 失败/不足
uint32 ca_drop_dup					# 累计因 timestamp 重复被丢弃的样本

# --- 回传侧（仅 FS_UT_FB=1 有效；否则下面字段为 NaN/0）---
uint8 fb_enabled					# 1 = FS_UT_FB 开启
float32[8] fb_angle_deg				# 实测角度；超时/未启用为 NaN
float32[8] fb_query_hz				# 每通道查询成功 Hz（1s 窗口）
uint8 online_flags					# bit i：通道 i 最近一次查询成功
uint32 query_ok						# 成功查询累计
uint32 query_err					# 校验/超时累计
float32 rtt_ema_us					# 问答 RTT 指数滑动均值
float32 rtt_max_us					# 1s 窗口峰值
