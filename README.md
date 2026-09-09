# PX4 电机 / 舵机辨识（本分支说明）

本仓库在 [PX4 Autopilot](https://github.com/PX4/PX4-Autopilot) 基础上增加了**多旋翼单电机推力辨识**、**FashionStar UART 总线舵机自适应扫频辨识**与 **PWM 舵机自适应扫频辨识**相关逻辑，便于台架标定与日志分析。

---

## 克隆与编译

本仓库依赖多个 Git 子模块（含 NuttX、驱动与仿真工具等）。**克隆后请先初始化子模块再编译**，否则可能出现 NuttX `dirlinks` 等构建错误。

```bash
git clone <本仓库 URL>
cd PX4_moto_iden
git submodule update --init --recursive
make px4_fmu-v5_default
```

若已克隆但未拉取子模块，在仓库根目录执行 `git submodule update --init --recursive` 即可。子模块状态异常或增量编译链接失败时，可尝试 `make distclean` 后重新编译。

固件输出：`build/px4_fmu-v5_default/px4_fmu-v5_default.px4`。更完整的工具链与烧录说明见 **[PX4 User Guide](https://docs.px4.io/)**。

### 在 WSL2 中直接刷写固件（`make ... upload`）

本分支移植了 PX4 main（v1.18 起，[PR #27674](https://github.com/PX4/PX4-Autopilot/pull/27674)）的 WSL2 上传支持：CMake 检测到 WSL2 时，`upload` 目标改用 **Windows 侧 `python.exe`** 运行 `Tools/px4_uploader.py`，直接枚举 Windows COM 口，无需 usbipd 透传，飞控进入 bootloader 重新枚举后也能自动接上。

```bash
make px4_fmu-v5_default upload          # 编译并刷写
make px4_fmu-v5_default force-upload    # 忽略 board_id 校验
make px4_fmu-v5_default upload-verbose  # 打印端口探测与协议细节
```

前置条件：

- Windows 已安装 Python 3（`python.exe` 在 PATH 中，Microsoft Store 版即可），并安装 pyserial：`python.exe -m pip install pyserial`
- 飞控 USB 直连电脑；Windows 侧 QGroundControl 等占用该 COM 口的程序需先断开
- 脚本会显示 `Waiting for bootloader...` 并轮询所有 COM 口，此时插上或复位飞控即可开始刷写；`Ctrl+C` 退出

原生 Linux / macOS 行为不变（使用 WSL/本机 Python 与 `/dev/serial/by-id`、`/dev/ttyACM*` 自动探测）。旧版 `Tools/px_uploader.py` 保留未动。

本分支已对 `px4_fmu-v5_default` 做**台架辨识专用裁剪**（Flash 约 **60%**，原约 89%）：保留 EKF2 姿态（`Servo_iden_data` 的 roll/pitch）、RC、USB/TELEM1 MAVLink、px4io MAIN + DShot、PWM AUX 与 `fs_uart_servo`；关闭 GPS、UAVCAN、位置控制/导航/自动模式等运行时模块。`navigator` 与 `mc_pos_control` 仅编译进固件以提供参数定义，**开机不启动**。

---

## 使用前准备

- **控制分配**：`SYS_CTRL_ALLOC = 1`，确保 `control_allocator` 开机自启（否则需手动 `control_allocator start`）。
- **日志**：`SDLOG_PROFILE` 包含 **Default**（默认值为 1）时，默认只记辨识相关话题（`Identify_data`、`Servo_iden_data`、`Fs_data`、执行器、门控）。姿态用 `Servo_iden_data.roll_angle` / `pitch_angle`（发送时刻写入），**不记** `vehicle_attitude` / `vehicle_attitude_setpoint`。EKF/GPS/`sensor_combined` 等已在 `logged_topics.cpp` 中注释掉。若使用 SD 卡上的 `logger_topics.txt` 自定义列表，需自行包含上述话题。
- **总线舵机辨识（IDEN_TYPE=3）**：
  - 板级已启用 `fs_uart_servo`（fmu-v5：`CONFIG_DRIVERS_FS_UART_SERVO=y`）。
  - 将 **`IDEN_TYPE` 设为 3** 后，`control_allocator` 会**自动启动** `fs_uart_servo`；改为其它辨识模式时自动关闭。
  - 默认占用板级 **TELEM2** 串口，请确保该串口未同时跑 MAVLink。
  - 总线舵机辨识扫频期间**自动开启回传**：只查 `IDEN_SV_IDX`，频率与发送相同（**`IDEN_SV_HZ`，默认 200 Hz**），写入 `Servo_iden_data.fb_angle`，不必再开 `FS_UT_FB`。
  - 若平时就要在 `Fs_data` 里看各路角度，再设 **`FS_UT_FB = 1`**。1 Mbaud 下每路目标 100 Hz（合计 800 次查询/秒），控制同步帧最高 200 Hz；低于 1 Mbaud 自动退回每路约 10 Hz。`Fs_data` 最高 200 Hz 发布。
  - **接线模式 `FS_UT_1WIRE`**（改后需重启）：`1`（默认）= TELEM2 **TX 单线直连**舵机 S 信号（UART 半双工，RX 不接）；`0` = 使用 UC-01 等独立 TX/RX 半双工转接板。
  - `FS_UT_INTV = 0` 使用 FashionStar 官方极速同步指令；大于 0 使用指定周期指令，此时 `FS_UT_TAC` / `FS_UT_TDC` 生效。
- **PWM 舵机辨识（IDEN_TYPE=4）**：
  - 依赖标准 **PWM 输出**（`pwm_out` / `io_bypass_pwm_servo` 等），在 QGC **Actuators** 中将对应通道映射为 **Servo1…Servo8**（`PWM_MAIN_FUNCx` / `PWM_AUX_FUNCx`）。
  - 此模式下 `fs_uart_servo` **保持关闭**（由 `control_allocator` 按 `IDEN_TYPE` 统一管理）。

---

## 参数（QGC 参数页「电机/舵机辨识」组）

| 参数 | 含义 |
|------|------|
| **IDEN_TYPE** | `0` 关闭；`1` 电机**定时升档**；`2` 电机 **AUX2 上升沿升档**；`3` **总线舵机扫频**；`4` **PWM 舵机扫频** |
| **IDEN_MOTOR_IDX** | 电机辨识目标电机序号（1 基，对应混控电机顺序），`IDEN_TYPE=1/2` 时使用 |
| **IDEN_SV_IDX** | 舵机辨识目标通道下标（0–7，对应 `actuator_servos.control[]`）。`IDEN_TYPE=3` 为 FashionStar 总线舵机 ID；`IDEN_TYPE=4` 为 PWM Servo 通道（需与 `PWM_*_FUNCx` 映射一致），默认 1 |
| **IDEN_SV_HZ** | 总线舵机辨识（`IDEN_TYPE=3`）发送频率（Hz），默认 200；回传自动与之相同。与 `FS_UT_R_HZ` 无关。PWM 辨识不用 |
| **IDEN_STEP_TIME** | 模式 1 下每一档保持时间（秒），默认 5 |
| **IDEN_AUX_THR** | 舵机辨识（`IDEN_TYPE=3/4`）时 AUX1 门限：需 `aux1 > IDEN_AUX_THR`（默认 0.3）；电机辨识仍使用 `aux1 > 0` |
| **IDEN_CH_F0** | 扫频起始频率（Hz），默认 0.2 |
| **IDEN_CH_F1** | 扫频上限（Hz），默认 30；实际取 $\min(F1, f_{1,\mathrm{eff}})$ |
| **IDEN_CH_DUR** | 单次对数扫频时长（秒），默认 90 |
| **IDEN_CH_AMP** | 低频幅值 $A_0$（度），默认 8 |
| **IDEN_CH_AMIN** | 最小幅值（度），默认 1.5；触底对应有效上限 $f_{1,\mathrm{eff}}$ |
| **IDEN_CH_STEP** | 速率自测阶跃幅值（度），默认 15 |
| **IDEN_CH_REP** | 全幅扫频重复次数，默认 3（用于相干函数） |
| **IDEN_CH_HALF** | 是否追加半幅扫频做线性性检验，默认 1（开） |

---

## 门控与 RC

- **飞行模式**：代码 `identify_rc_mode_active()` 允许 Manual / Stabilized / Acro / Altitude / Position，但裁剪固件**未启动** `mc_pos_control` 与 `flight_mode_manager`，实际请使用 **Manual（手动）**、**Stabilized（自稳）** 或 **Acro（特技）** 进行辨识；定高/定点模式无位置控制器支撑。
- **AUX1**：
  - 电机辨识（`IDEN_TYPE=1/2`）：`manual_control_setpoint.aux1 > 0` 时允许进入辨识流程。
  - 舵机辨识（`IDEN_TYPE=3/4`）：`aux1 > IDEN_AUX_THR` 时允许进入自适应扫频。
  - 需在地面站配置 **RC_MAP_AUX1**。
- **解锁、非 kill** 等安全条件需满足后状态机才运行。
- **辨识次数**：同一次解锁最多跑完一轮；跑完后输出保持 0，再拨 AUX1 不会重开。上锁后完成标志清除，再次解锁可再辨识（可先改 `IDEN_SV_IDX` / `IDEN_MOTOR_IDX`）。

---

## 电机辨识（IDEN_TYPE=1 / 2）

- 推力指令从 **0.1** 到 **1.0**，步长 **0.05**（共 19 个档位：0.1, 0.15, …, 1.0）。
- **模式 1（IDEN_TYPE=1）**：按 `IDEN_STEP_TIME` **定时**自动升档。
- **模式 2（IDEN_TYPE=2）**：**仅当 AUX2 从 ≤0 变为 >0**（上升沿）时升一档；**从 >0 变为 ≤0** 时不升不降，保持当前档。需在地面站配置 **RC_MAP_AUX2**。
- **每次解锁**完整跑完一轮后：标记完成，电机输出**保持为 0**（直至上锁）；同一解锁周期内不重复整段阶跃。上锁后再解锁，可再跑一轮。

---

## 总线舵机扫频（IDEN_TYPE=3）

一次 AUX1 触发自动跑完整序列（门控断开则从头开始；**同一次解锁内**跑完后不再重复，上锁后再解锁可再跑）：

1. **静置** 1 s（0°）
2. **速率自测** 约 4 s：$0 \rightarrow +S \rightarrow -S \rightarrow 0$（$S=$ `IDEN_CH_STEP`）。优先用陀螺峰值（$>30^\circ/\mathrm{s}$ 视为 IMU 装在舵臂上），否则用总线回传角三点斜率，得到 $V_{\max}$。都没有则 `fb_source=0`，退化为固定幅值扫到 `IDEN_CH_F1`。
3. **引导**：$f_0$ 恒频 2 个整周期
4. **对数扫频**：$f_0 \rightarrow f_{1,\mathrm{use}}$，时长 `IDEN_CH_DUR`，结束过零收尾；重复 `IDEN_CH_REP` 次（段间停 2 s）
5. **半幅扫频**（`IDEN_CH_HALF=1`）：幅值减半再扫一次，用于线性性检验

幅值随频率衰减（速率裕度 $k=0.6$ 写死）：

$$
A(f)=\mathrm{clamp}\!\left(\frac{k\,V_{\max}}{2\pi f},\,A_{\min},\,A_0\right),\quad
f_{1,\mathrm{eff}}=\frac{k\,V_{\max}}{2\pi A_{\min}},\quad
f_{1,\mathrm{use}}=\min(F_1,\,f_{1,\mathrm{eff}})
$$

默认参数下全流程约 **7 分钟**（`REP=3`、`HALF=1`、`F0=0.2`、`DUR=90`）。台架时间紧可设 `IDEN_CH_REP=1`、`IDEN_CH_HALF=0`，约 **1.8 分钟**。

- **波形生成**：在 `fs_uart_servo` **发送时刻**（节拍 $1/f_{\mathrm{ctrl}}$，`IDEN_SV_HZ`，默认 5 ms）计算，绕过 `FS_UT_GAIN` / `FS_UT_TRM`，直接写入同步帧；CA 只发布门控话题 `Servo_iden_ctrl`，`actuator_servos` 保持 0。
- **回传**：扫频一开始自动查询 `IDEN_SV_IDX`，与同步帧同频（`IDEN_SV_HZ`）；结束后若 `FS_UT_FB=0` 则关掉回传。
- **输出逻辑**：仅目标舵机动作，其余通道 0°；结束后全部保持 0° 直至**上锁**。
- **驱动**：`IDEN_TYPE=3` 时自动启动 `fs_uart_servo`；切到其它模式自动关闭。

---

## PWM 舵机扫频（IDEN_TYPE=4）

- **激励序列**：与 type 3 共用 `ServoIdenSequencer`（静置 / 速率自测 / 引导 / 扫频 / 半幅），由 `control_allocator` 以 5 ms 节拍推进。
- **速率自测**：仅能用陀螺（飞控需装在舵臂上）。测不到则 `fb_source=0`，固定 `IDEN_CH_AMP` 扫到 `IDEN_CH_F1`。
- **PWM 映射**：内部以度计算，写入 PWM 前按 PX4_Tilt 控制分配方式换算：
  - \(u_{\mathrm{pwm}} = 1.27 \cdot \theta_{\mathrm{rad}}\)（约 \(4/\pi\)，**±45°** 对应满舵 \([-1,1]\)）
  - ±8° 约对应 ±0.177 归一化，不会打满舵
- **输出逻辑**：门控满足时，仅 **IDEN_SV_IDX** 对应 PWM Servo 通道输出，其余通道置 **0**（中位）；完成后全部保持 0 直至上锁。上锁后再解锁可再跑一轮。
- **日志**：`Servo_iden_data.chirp_signal` 为 **[-1,1] 归一化**；`cmd_deg` 为度；`fb_angle` 为 NAN。
- **驱动**：`fs_uart_servo` 在此模式下保持关闭。

---

## uORB 话题

| 话题 | 说明 |
|------|------|
| **`Identify_data`** | 电机辨识：由 `control_allocator` 发布，含 `motor_idx`、`identify_active`、`cmd_norm`（归一化指令）、`pwm_out` 等 |
| **`Servo_iden_data`** | 舵机辨识：`iden_active`、`servo_idx`、`chirp_signal`（type3 为度，type4 为 [-1,1]）、`cmd_deg`（统一为度）、`phase`（0 IDLE … 7 DONE）、`rep_idx`、`sweep_t`、`inst_freq_hz`、`amp_deg`、`vmax_dps`、`f1_eff_hz`、`fb_source`（0 无反馈 / 1 陀螺 / 2 总线回传）、`gyro_dps[3]`、`roll_angle` / `pitch_angle`（发送时刻姿态角，度）、`fb_angle`（仅 type3）。type 3 的 `timestamp` 为同步帧写出时刻。 |
| **`Servo_iden_ctrl`** | CA → `fs_uart_servo` 门控：`enable`、`servo_idx`（不记日志） |
| **`Fs_data`** | `fs_uart_servo` 驱动发布：命令角、总线统计；`FS_UT_FB=1` 或辨识自动回传时含各通道回传角度 `fb_angle_deg[]` |

---

## 主要改动位置（开发参考）

- `src/modules/control_allocator/ControlAllocator.cpp` / `.hpp`：电机辨识、舵机门控（type 3）与 PWM 扫频（type 4）、话题发布。
- `src/lib/servo_iden/`：自适应扫频状态机（总线 / PWM 共用）。
- `msg/identify_data.msg`、`msg/Servo_iden_data.msg`、`msg/Servo_iden_ctrl.msg`：辨识消息定义。
- `src/drivers/fs_uart_servo/`：FashionStar UART 总线舵机驱动（type 3 在发送节拍生成波形，发布 `Fs_data` / `Servo_iden_data`）。
- `msg/Fs_data.msg`：舵机驱动状态与回传数据。
- `src/modules/logger/logged_topics.cpp`：默认记录 `Identify_data`、`Servo_iden_data`、`Fs_data`；其余无关话题已注释。
- `src/modules/commander/commander_params.c`：`IDEN_*` 参数定义（`@group 电机/舵机辨识`，在 QGC 中独立成组）。
- `boards/px4/fmu-v5/default.px4board`：辨识专用裁剪（保留 EKF2/RC/MAVLink/px4io/DShot/fs_uart_servo；关 GPS/UAVCAN/位置导航等）。
- `ROMFS/px4fmu_common/init.d/rc.mc_apps`：固定 `ekf2` + 手飞控制链 + `control_allocator`；不启动 navigator / mc_pos_control / flight_mode_manager。
- `ROMFS/px4fmu_common/init.d/rc.mc_defaults`：`SYS_CTRL_ALLOC=1`、`MIXER skip`（control_allocator 模式）。

---

## 上游 PX4

通用编译、烧录、机型与官方文档见：**[PX4 User Guide](https://docs.px4.io/)**。

License 等以仓库内 `LICENSE` 及上游 [PX4/PX4-Autopilot](https://github.com/PX4/PX4-Autopilot) 为准。
