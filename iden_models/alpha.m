% alpha 舵机：指令角 → 总线回传角 闭环二阶模型（100 Hz 发令，log15/16/17/2 合并拟合）
% 输入输出：度。
% Td 选法：仿真里指令已按 100 Hz 零阶保持再进连续模型 -> 用 alpha.Td_plant (4 ms)；
%          把 100 Hz 指令序列当连续信号、或 c2d 后直接比对实测 Bode -> 用 alpha.Td_total (9 ms)。

alpha.K = 0.97;
alpha.fn_hz = 9.5;
alpha.zeta = 0.52;
alpha.Td_total = 0.009;
alpha.Td_plant = 0.004;
alpha.Td = alpha.Td_plant;
alpha.wn = 2 * pi * alpha.fn_hz;
alpha.tf = alpha.K * tf(alpha.wn^2, [1, 2 * alpha.zeta * alpha.wn, alpha.wn^2], ...
    'InputDelay', alpha.Td);

% 100 Hz 精确 ZOH 离散等效（含 4 ms 分数延迟），可直接用于离散仿真
alpha.Ts = 0.01;
alpha.tf_d = tf([0 0.054582 0.179289 0.016094], [1 -1.279830 0.537526 0], alpha.Ts);

% 非线性：速率限制约 555 deg/s，±15° 阶跃受速率/加速度整形（实测超调远小于线性），
% 线性模型只在 8° 以内的扫频上验证过。
alpha.vmax_dps = 555;
alpha.valid_to_hz = 12;
