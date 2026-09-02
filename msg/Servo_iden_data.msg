uint64 timestamp

bool iden_active           # 辨识是否运行中
uint8 servo_idx            # actuator_servos.control[] 下标（0–7）
float32 chirp_signal       # 当前 chirp 指令：IDEN_TYPE=3 为度；IDEN_TYPE=4 为 PWM 归一化 [-1,1]

float32 roll_angle         # IMU 滚转角（度），来自 vehicle_attitude
float32 pitch_angle        # IMU 俯仰角（度），来自 vehicle_attitude
float32 fb_angle           # 总线舵机回传角度（度，IDEN_TYPE=3 且 FS_UT_FB=1）；否则 NAN
