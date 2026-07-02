# PX4 电机 / 舵机辨识（本分支说明）

本仓库在 [PX4 Autopilot](https://github.com/PX4/PX4-Autopilot) 基础上增加了**多旋翼单电机推力辨识**与**FashionStar UART 舵机 chirp 辨识**相关逻辑，便于台架标定与日志分析。

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

---

## 使用前准备

- **控制分配**：`SYS_CTRL_ALLOC = 1`，确保 `control_allocator` 开机自启（否则需手动 `control_allocator start`）。
- **日志**：`SDLOG_PROFILE` 包含 **Default**（默认值为 1）时，`Identify_data`、`Servo_iden_data`、`Fs_data` 会进入飞行日志；若使用 SD 卡上的 `logger_topics.txt` 自定义列表，需自行包含上述话题。
- **舵机驱动（仅舵机辨识）**：
  - 板级已启用 `fs_uart_servo`（fmu-v5：`CONFIG_DRIVERS_FS_UART_SERVO=y`）。
  - 设置 **`FS_UT_BOOT = 1`** 可在多旋翼启动脚本中自动执行 `fs_uart_servo start`；也可手动启动。
  - 默认占用板级 **TELEM2** 串口，请确保该串口未同时跑 MAVLink。
  - 若需舵机回传角度写入日志，设置 **`FS_UT_FB = 1`**（见 `Fs_data` 话题）。

---

## 参数（Commander 组）

| 参数 | 含义 |
|------|------|
| **IDEN_TYPE** | `0` 关闭；`1` 电机**定时升档**；`2` 电机 **AUX2 上升沿升档**；`3` 舵机 **chirp 扫频**（见下节） |
| **IDEN_MOTOR_IDX** | 电机辨识目标电机序号（1 基，对应混控电机顺序），`IDEN_TYPE=1/2` 时使用 |
| **IDEN_SV_IDX** | 舵机辨识目标 **FashionStar 总线舵机 ID**（0–7，与舵机机身 ID、`fs_uart_servo` 协议 `servo_id` 一致；设 1 只动 1 号，设 2 只动 2 号），`IDEN_TYPE=3` 时使用，默认 1 |
| **IDEN_STEP_TIME** | 模式 1 下每一档保持时间（秒），默认 5 |
| **IDEN_TRIG_THR / IDEN_TRIG_TIME** | 油门触发相关（参数保留，按当前固件逻辑以 AUX/门控为准） |
| **IDEN_AUX_THR** | 舵机辨识（`IDEN_TYPE=3`）时 AUX1 门限：需 `aux1 > IDEN_AUX_THR`（默认 0.3）；电机辨识仍使用 `aux1 > 0` |

---

## 门控与 RC

- **飞行模式**：手飞类（自稳 / 定高 / 定位置 / 特技等，见代码 `identify_rc_mode_active()`）。
- **AUX1**：
  - 电机辨识（`IDEN_TYPE=1/2`）：`manual_control_setpoint.aux1 > 0` 时允许进入辨识流程。
  - 舵机辨识（`IDEN_TYPE=3`）：`aux1 > IDEN_AUX_THR` 时允许进入 chirp 扫频。
  - 需在地面站配置 **RC_MAP_AUX1**。
- **解锁、非 kill** 等安全条件需满足后状态机才运行。

---

## 电机辨识（IDEN_TYPE=1 / 2）

- 推力指令从 **0.1** 到 **1.0**，步长 **0.05**（共 19 个档位：0.1, 0.15, …, 1.0）。
- **模式 1（IDEN_TYPE=1）**：按 `IDEN_STEP_TIME` **定时**自动升档。
- **模式 2（IDEN_TYPE=2）**：**仅当 AUX2 从 ≤0 变为 >0**（上升沿）时升一档；**从 >0 变为 ≤0** 时不升不降，保持当前档。需在地面站配置 **RC_MAP_AUX2**。
- **每机上电**完整跑完一轮后：标记完成，电机输出**保持为 0**（直至重启）；同一上电周期内不重复整段阶跃。

---

## 舵机辨识（IDEN_TYPE=3）

- **激励信号**：对数扫频 chirp，开环直接写入 `actuator_servos.control[]` 目标通道（单位：**度**）。
  - 频率：**0.01 → 80 Hz**
  - 时长：**120 s**
  - 幅值：**±8°**（中心 0°）
- **输出逻辑**：门控满足时，仅 **IDEN_SV_IDX 指定的总线舵机** 输出 chirp，**其余通道置 0**；chirp 结束后标记完成，**所有舵机输出保持 0**（直至重启）。
- **每机上电**完整跑完一轮 chirp 后，同一上电周期内不重复扫频。

---

## uORB 话题

| 话题 | 说明 |
|------|------|
| **`Identify_data`** | 电机辨识：由 `control_allocator` 发布，含 `motor_idx`、`identify_active`、`cmd_norm`（归一化指令）、`pwm_out` 等 |
| **`Servo_iden_data`** | 舵机辨识：含 `iden_active`、`servo_idx`、`chirp_signal`（度）、`roll_angle` / `pitch_angle`（度）、`fb_angle`（舵机回传角度，度；无回传时为 NAN） |
| **`Fs_data`** | `fs_uart_servo` 驱动发布：命令角、总线统计；`FS_UT_FB=1` 时含各通道回传角度 `fb_angle_deg[]` |

---

## 主要改动位置（开发参考）

- `src/modules/control_allocator/ControlAllocator.cpp` / `.hpp`：电机 / 舵机辨识状态机、执行器覆盖、话题发布。
- `msg/identify_data.msg`、`msg/Servo_iden_data.msg`：辨识消息定义。
- `src/drivers/fs_uart_servo/`：FashionStar UART 总线舵机驱动（`Fs_data` 发布）。
- `msg/Fs_data.msg`：舵机驱动状态与回传数据。
- `src/modules/logger/logged_topics.cpp`：默认记录 `Identify_data`、`Servo_iden_data`、`Fs_data`。
- `src/modules/commander/commander_params.c`：`IDEN_*` 参数定义。
- `boards/px4/fmu-v5/default.px4board`：启用 `fs_uart_servo`。
- `ROMFS/px4fmu_common/init.d/rc.mc_apps`：`FS_UT_BOOT=1` 时自启 `fs_uart_servo`。

---

## 上游 PX4

通用编译、烧录、机型与官方文档见：**[PX4 User Guide](https://docs.px4.io/)**。

License 等以仓库内 `LICENSE` 及上游 [PX4/PX4-Autopilot](https://github.com/PX4/PX4-Autopilot) 为准。
