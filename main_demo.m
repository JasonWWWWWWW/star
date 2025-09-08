% main_demo.m  主演示脚本：读取 IMU -> 标定 -> 姿态 -> INS+KF -> 绘图
clear; close all; clc;

% ------------ 用户配置 -------------
dataPath = "merged_sensor_data.csv";   % CSV: [t, ax, ay, az, gx, gy, gz]
fs = [];
deg2rad = pi/180;

% KF 参数（推荐稳健的起点）
params.g = 9.80665;
params.Q_gyro_bias = (0.005*deg2rad)^2;
params.Q_acc_bias  = (0.05)^2;
params.Q_vel       = 1e-4;
params.R_zupt_acc  = 1.0;    % 更保守一些，避免过度自信

% Madgwick 参数
beta = 0.8;

% ------------- 读取数据 --------------
if ~isfile(dataPath)
    error("找不到数据文件：%s，请确认路径。", dataPath);
end
D = readmatrix(dataPath);
if size(D,2) < 7
    error("CSV 格式应为 [t, ax, ay, az, gx, gy, gz]");
end
t = D(:,1);

% 原始列（请确保你的轴映射/正负号正确）
ax_raw = D(:,2); ay_raw = D(:,3); az_raw = D(:,4);

% 注意：以下映射仅为当前数据的示例，如与你的手机坐标不同，请修改
ax = - D(:,3);
ay = - D(:,2);
az = - D(:,1);

gx_raw = D(:,5); gy_raw = D(:,6); gz_raw = D(:,7);
gx = - D(:,7); gy = - D(:,6); gz = - D(:,5); % 假定手机输出已为 rad/s；若为 deg/s 请乘以 deg2rad

if isempty(fs)
    dt_vec = diff(t);
    dt = median(dt_vec);
else
    dt = 1/fs;
end
N = length(t);

% ------------ 标定 -------------
[ax_c, ay_c, az_c, gx_c, gy_c, gz_c, calib] = calibrate_imu(ax,ay,az,gx,gy,gz, t);
fprintf("标定完成：加速度偏置 [%g %g %g], 陀螺偏置 [%g %g %g]\n", calib.acc_bias, calib.gyro_bias);

% 将标定后的传感器送入姿态/INS
acc_body = [ax_c ay_c az_c];
gyro_body = [gx_c gy_c gz_c];

% ------------ 姿态估计（Madgwick） -------------
q = zeros(N,4);
useN = min(N, max(10, round(1/median(diff(t)))));
a0 = [mean(ax_c(1:useN)), mean(ay_c(1:useN)), mean(az_c(1:useN))];
roll0  = atan2( a0(2), a0(3) );
pitch0 = atan2( -a0(1), sqrt(a0(2)^2 + a0(3)^2) );
yaw0 = 0;
cy = cos(yaw0*0.5); sy = sin(yaw0*0.5);
cp = cos(pitch0*0.5); sp = sin(pitch0*0.5);
cr = cos(roll0*0.5); sr = sin(roll0*0.5);
q0w = cr*cp*cy + sr*sp*sy;
q0x = sr*cp*cy - cr*sp*sy;
q0y = cr*sp*cy + sr*cp*sy;
q0z = cr*cp*sy - sr*sp*cy;
q(1,:) = [q0w q0x q0y q0z];

for k=2:N
    dtk = t(k) - t(k-1);
    q(k,:) = madgwick_ahrs( acc_body(k,:), gyro_body(k,:), q(k-1,:), beta, dtk );
    if mod(k,200)==0, q(k,:) = q(k,:)/norm(q(k,:)); end
end
for k=1:N, q(k,:) = q(k,:)/norm(q(k,:)); end

% ------------ 转加速度到导航系并扣重力（仅用于可视化） -------------
acc_nav = zeros(N,3);
for k=1:N
    Cbn = quat2dcm(q(k,:));
    a_nav = Cbn * acc_body(k,:)';
    a_nav = a_nav' - [0 0 params.g];
    a_nav(a_nav > 200) = 200; a_nav(a_nav < -200) = -200;
    acc_nav(k,:) = a_nav;
end

% ------------ ZUPT 检测 -------------
is_still = zupt_detect(acc_body, gyro_body, dt);

% ------------ 计算初始 body-frame 加速度偏置 -------------
still_idx = find(is_still);
if ~isempty(still_idx)
    idxs = still_idx(1:min(length(still_idx), round(0.5/dt)));
    mean_acc_body_still = mean(acc_body(idxs,:),1)';
    Csum = zeros(3,3);
    for ii = idxs'
        Csum = Csum + quat2dcm(q(ii,:));
    end
    Cmean = Csum / length(idxs);
    g_body_pred = Cmean' * [0;0;params.g];
    init_bias_body = mean_acc_body_still - g_body_pred;     % body-frame 偏置估计
    fprintf('Computed init_bias_body from first still window = [%.4f %.4f %.4f]\n', init_bias_body);
else
    init_bias_body = [0;0;0];
end

opts_kf = struct();
opts_kf.debug = true;
% KF options
opts_kf.quick_bias_corr = false;
opts_kf.bias_corr_alpha = 0.18;
opts_kf.cross_cov_scale = 0.05;
opts_kf.init_bias_scale = 1.0;        % 与 init_bias 一起使用（为 1 则完全采用）
opts_kf.init_bias = init_bias_body;   % 直接注入初始 body-frame bias
opts_kf.init_bias_P_scale = 0.02;     % 将 acc_bias 初始方差缩小
opts_kf.VEL_INNOV_GATE = 1.0;
opts_kf.MAX_DX_VEL = 2.0;
opts_kf.MAX_DX_BIAS = 2.5;
opts_kf.MAX_DX_POS = 2.0;
opts_kf.bias_meas_R = 0.05;

% ------------- 调用 KF（传入 body-frame 加速度序列） -------------
[state_hist, P_hist, diagnostics] = ins_propagate_kf(acc_body, q, t, is_still, params, opts_kf);

% ------------- 绘图与诊断 -------------
plot_results(t, q, state_hist, acc_nav, is_still);
print_run_diagnostics();

if isfield(diagnostics,'mean_acc_still_list') && ~isempty(diagnostics.mean_acc_still_list)
    mlist = diagnostics.mean_acc_still_list;
    fprintf('每次ZUPT时 mean_a_nav 平均值 (first/mean/last) in m/s^2:\n');
    fprintf(' first: [%.3f %.3f %.3f]\n', mlist(1,:));
    fprintf(' mean : [%.3f %.3f %.3f]\n', mean(mlist,1));
    fprintf(' last : [%.3f %.3f %.3f]\n', mlist(end,:));
end

% 简洁输出几项关键统计
fprintf('diagnostics.zupt_scaled_count = %d\n', diagnostics.zupt_scaled_count);
fprintf('diagnostics.skipped_zupt = %d\n', diagnostics.skipped_zupt);
fprintf('diagnostics.quick_bias_applied = %d\n', diagnostics.quick_bias_applied);

fprintf("演示完成。\n");

