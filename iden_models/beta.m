% beta 舵机：指令角 → 总线回传角 闭环二阶模型（100 Hz 发令，log6 主拟合，log18/0/3/4 复核）
% 输入输出：度。
% Td 选法：仿真里指令已按 100 Hz 零阶保持再进连续模型 -> 用 beta.Td_plant (10.7 ms)；
%          把 100 Hz 指令序列当连续信号、或 c2d 后直接比对实测 Bode -> 用 beta.Td_total (15.7 ms)。

beta.K = 0.97;
beta.fn_hz = 24.5;
beta.zeta = 0.58;
beta.Td_total = 0.0157;
beta.Td_plant = 0.0107;
beta.Td = beta.Td_plant;
beta.wn = 2 * pi * beta.fn_hz;
beta.tf = beta.K * tf(beta.wn^2, [1, 2 * beta.zeta * beta.wn, beta.wn^2], ...
    'InputDelay', beta.Td);

% 100 Hz 精确 ZOH 离散等效（含 0.7 ms 分数延迟），可直接用于离散仿真
beta.Ts = 0.01;
beta.tf_d = tf([0 0 0.526823 0.357371 0.000984], [1 -0.255129 0.167683 0 0], beta.Ts);

% 非线性：速率限制约 600 deg/s，±15° 阶跃已饱和；4° 半幅扫频直流增益 0.92，约 0.2° 死区。
beta.vmax_dps = 600;
beta.valid_to_hz = 26;
