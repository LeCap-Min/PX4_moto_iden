uint64 timestamp

bool iden_active           # 辨识是否运行中
uint8 servo_idx            # 目标 FashionStar 总线舵机 ID
float32 chirp_signal       # 当前 chirp 指令值（度）

float32 roll_angle         # IMU 滚转角（度），来自 vehicle_attitude
float32 pitch_angle        # IMU 俯仰角（度），来自 vehicle_attitude
float32 fb_angle           # 舵机回传角度（度），无回传时为 NAN
