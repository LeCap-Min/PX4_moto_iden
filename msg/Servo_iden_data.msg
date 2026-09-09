uint64 timestamp

bool iden_active           # 辨识是否运行中
uint8 servo_idx            # actuator_servos.control[] 下标（0–7）
float32 chirp_signal       # 兼容字段：IDEN_TYPE=3 为度；IDEN_TYPE=4 为 PWM 归一化 [-1,1]

float32 roll_angle         # 发送时刻机体滚转角（度，EKF 四元数转欧拉）；后处理直接用本字段
float32 pitch_angle        # 发送时刻机体俯仰角（度，EKF 四元数转欧拉）；后处理直接用本字段
float32 fb_angle           # 总线舵机回传角度（度，IDEN_TYPE=3 扫频期间自动查询；否则 NAN）

float32 cmd_deg            # 当前指令角（度），type 3/4 统一
uint8 phase                # 0 IDLE 1 SETTLE 2 SLEW_TEST 3 LEAD_IN 4 SWEEP 5 PAUSE 6 SWEEP_HALF 7 DONE
uint8 rep_idx              # 全幅扫频 0 起；半幅扫频等于 IDEN_CH_REP
float32 sweep_t            # 当前引导/扫频段内时间（秒）
float32 inst_freq_hz       # 瞬时频率（Hz）
float32 amp_deg            # 当前包络幅值（度）
float32 vmax_dps           # 速率自测得到的峰值角速度（度/秒）；无反馈为 0
float32 f1_eff_hz          # 有效扫频上限（Hz）
uint8 fb_source            # 0 无反馈（固定幅值） 1 陀螺 2 总线回传角
float32[3] gyro_dps        # 机体角速度（度/秒）
