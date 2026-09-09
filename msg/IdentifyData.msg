uint64 timestamp

uint8 motor_idx
bool identify_active
float32 cmd_norm
float32 pwm_out                 # 执行器原始输出：DShot 时为油门计数（约 48–2047），PWM 时为脉宽

# 1.16 DShot 回传（esc_status：双向 DShot RPM 和/或 UART 遥测）
int32 esc_rpm                   # 电调回传转速 [RPM]；无回传为 0
float32 esc_voltage             # 电调回传电压 [V]；无则为 NAN
float32 esc_current             # 电调回传电流 [A]；无则为 NAN
float32 esc_temperature         # 电调回传温度 [degC]；无则为 NAN
uint8 esc_online                # 1 = esc_status 该路 online
uint8 dshot_tel_ok              # 1 = 该电机 500 ms 内有过 esc_status 更新

# TOPICS Identify_data
