# PX4 电机辨识（本分支说明）

本仓库在 [PX4 Autopilot](https://github.com/PX4/PX4-Autopilot) 基础上增加了**多旋翼单电机推力辨识**相关逻辑，便于台架标定与日志分析。

---

## 使用前准备

- **控制分配**：`SYS_CTRL_ALLOC = 1`，确保 `control_allocator` 开机自启（否则需手动 `control_allocator start`）。
- **日志**：`SDLOG_PROFILE` 包含 **Default**（默认值为 1）时，`Identify_data` 会进入飞行日志；若使用 SD 卡上的 `logger_topics.txt` 自定义列表，需自行包含 `Identify_data`。

---

## 参数（Commander 组）

| 参数 | 含义 |
|------|------|
| **IDEN_TYPE** | `0` 关闭；`1` **定时升档**（每档时间见 `IDEN_STEP_TIME`）；`2` **AUX2 上升沿升档**（见下节） |
| **IDEN_MOTOR_IDX** | 目标电机序号（1 基，对应混控电机顺序） |
| **IDEN_STEP_TIME** | 模式 1 下每一档保持时间（秒），默认 5 |
| **IDEN_TRIG_THR / IDEN_TRIG_TIME** | 油门触发相关（参数保留，按当前固件逻辑以 AUX/门控为准） |
| **IDEN_AUX_THR** | AUX1 门限相关（参数保留，可与 `RC_MAP_AUX1` 配合） |

---

## 门控与 RC

- **飞行模式**：手飞类（自稳 / 定高 / 定位置 / 特技等，见代码 `identify_rc_mode_active()`）。
- **AUX1**：`manual_control_setpoint.aux1 > 0` 时允许进入辨识流程（需在地面站配置 **RC_MAP_AUX1**）。
- **解锁、非 kill** 等安全条件需满足后状态机才运行。

---

## 辨识序列（两种模式共用）

- 推力指令从 **0.1** 到 **1.0**，步长 **0.05**（共 19 个档位：0.1, 0.15, …, 1.0）。
- **模式 1（IDEN_TYPE=1）**：按 `IDEN_STEP_TIME` **定时**自动升档。
- **模式 2（IDEN_TYPE=2）**：**仅当 AUX2 从 ≤0 变为 >0**（上升沿）时升一档；**从 >0 变为 ≤0** 时不升不降，保持当前档。需在地面站配置 **RC_MAP_AUX2**。
- **每机上电**完整跑完一轮后：标记完成，电机输出**保持为 0**（直至重启）；同一上电周期内不重复整段阶跃。

---

## uORB 话题

- **`Identify_data`**：由 `control_allocator` 发布，含 `motor_idx`、`identify_active`、`cmd_norm`（归一化指令）、`pwm_out`（目标路 PWM 等）等，用于记录与离线拟合。

---

## 主要改动位置（开发参考）

- `src/modules/control_allocator/ControlAllocator.cpp` / `.hpp`：辨识状态机、单电机覆盖、`Identify_data` 发布。
- `msg/identify_data.msg`：消息定义。
- `src/modules/logger/logged_topics.cpp`：默认记录 `Identify_data`。
- `src/modules/commander/commander_params.c`：`IDEN_*` 参数定义。

---

## 上游 PX4

通用编译、烧录、机型与官方文档见：**[PX4 User Guide](https://docs.px4.io/)**。

License 等以仓库内 `LICENSE` 及上游 [PX4/PX4-Autopilot](https://github.com/PX4/PX4-Autopilot) 为准。
