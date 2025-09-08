% main_demo_improved.m - Enhanced dumbbell INS with better calibration and drift control
clear; close all; clc;

% ------------ Enhanced User Configuration -------------
dataPath = "merged_sensor_data.csv";
fs = [];
deg2rad = pi/180;

% Enhanced KF parameters for better stability
params.g = 9.80665;
params.Q_gyro_bias = (0.002*deg2rad)^2;  % Reduced for more stable gyro bias
params.Q_acc_bias  = (0.02)^2;           % Reduced for more stable acc bias
params.Q_vel       = 5e-5;               % Reduced velocity process noise
params.R_zupt_acc  = 0.5;                % Balanced observation noise

% Enhanced Madgwick parameters
beta = 0.6;  % Slightly reduced for smoother attitude estimation

% ------------- Read and preprocess data --------------
if ~isfile(dataPath)
    error("找不到数据文件：%s，请确认路径。", dataPath);
end
D = readmatrix(dataPath);
if size(D,2) < 7
    error("CSV 格式应为 [t, ax, ay, az, gx, gy, gz]");
end
t = D(:,1);

% Apply coordinate transformation and filtering
ax_raw = D(:,2); ay_raw = D(:,3); az_raw = D(:,4);
gx_raw = D(:,5); gy_raw = D(:,6); gz_raw = D(:,7);

% Enhanced coordinate mapping with filtering
[ax, ay, az, gx, gy, gz] = preprocess_imu_data(D);

if isempty(fs)
    dt_vec = diff(t);
    dt = median(dt_vec);
else
    dt = 1/fs;
end
N = length(t);

% ------------ Enhanced Calibration -------------
[ax_c, ay_c, az_c, gx_c, gy_c, gz_c, calib] = calibrate_imu_enhanced(ax,ay,az,gx,gy,gz, t);
fprintf("增强标定完成：\n");
fprintf("  加速度偏置: [%.4f %.4f %.4f] m/s²\n", calib.acc_bias);
fprintf("  陀螺偏置: [%.4f %.4f %.4f] deg/s\n", calib.gyro_bias*180/pi);
fprintf("  加速度尺度因子: [%.4f %.4f %.4f]\n", calib.acc_scale);

% ------------ Enhanced Attitude Estimation -------------
q = zeros(N,4);
q = estimate_attitude_enhanced(ax_c, ay_c, az_c, gx_c, gy_c, gz_c, t, beta);

% ------------ Enhanced ZUPT Detection -------------
zupt_opts.window = 0.3;              % Longer window for stability
zupt_opts.min_still_time = 0.8;      % Longer minimum still time
zupt_opts.acc_factor = 3;            % More conservative
zupt_opts.gyro_factor = 3;           % More conservative
zupt_opts.use_percentile = true;
zupt_opts.percentile_level = 15;     % Higher percentile for robustness

is_still = zupt_detect_enhanced([ax_c ay_c az_c], [gx_c gy_c gz_c], dt, zupt_opts);

% ------------ Enhanced Initial Bias Estimation -------------
init_bias_body = estimate_initial_bias_enhanced(ax_c, ay_c, az_c, q, is_still, params);
fprintf('增强初始偏置估计 (body-frame): [%.4f %.4f %.4f] m/s²\n', init_bias_body);

% ------------ Enhanced KF Options -------------
opts_kf = struct();
opts_kf.debug = true;
opts_kf.quick_bias_corr = true;
opts_kf.bias_corr_alpha = 0.08;           % Moderate correction rate
opts_kf.cross_cov_scale = 0.02;           % Stronger decoupling
opts_kf.init_bias_scale = 1.0;            % Use full initial bias
opts_kf.init_bias = init_bias_body;
opts_kf.init_bias_P_scale = 0.01;         % Confident in initial bias
opts_kf.VEL_INNOV_GATE = 2.5;             % More permissive gate
opts_kf.MAX_DX_VEL = 1.5;                 % Smaller velocity updates
opts_kf.MAX_DX_BIAS = 1.0;                % Moderate bias updates
opts_kf.MAX_DX_POS = 1.0;                 % Smaller position updates
opts_kf.bias_meas_R = 0.02;               % Trust bias measurements more
opts_kf.TRUST_BIAS_MEAS_ANGLE_DEG = 20.0; % More lenient angle threshold
opts_kf.BIAS_WINDOW_MAX_STD = 1.0;        % Stricter stability requirement
opts_kf.drift_compensation = true;        % Enable drift compensation
opts_kf.velocity_damping = 0.98;          % Slight velocity damping during motion

% ------------- Enhanced INS with KF -------------
acc_body = [ax_c ay_c az_c];
[state_hist, P_hist, diagnostics] = ins_propagate_kf_enhanced(acc_body, q, t, is_still, params, opts_kf);

% ------------- Enhanced Results and Analysis -------------
plot_results_enhanced(t, q, state_hist, is_still, diagnostics);
print_enhanced_diagnostics(diagnostics, t, state_hist);

fprintf("\n=== 性能总结 ===\n");
final_pos = state_hist(end,1:3);
final_vel = state_hist(end,4:6);
max_pos_drift = max(abs(final_pos));
max_vel_drift = max(abs(final_vel));

fprintf("最终位置: [%.2f %.2f %.2f] m\n", final_pos);
fprintf("最终速度: [%.2f %.2f %.2f] m/s\n", final_vel);
fprintf("最大位置漂移: %.2f m\n", max_pos_drift);
fprintf("最大速度漂移: %.2f m/s\n", max_vel_drift);

if max_pos_drift < 20 && max_vel_drift < 2
    fprintf("✓ 性能良好\n");
elseif max_pos_drift < 50 && max_vel_drift < 5
    fprintf("⚠ 性能中等，需要进一步优化\n");
else
    fprintf("✗ 性能较差，需要重新检查参数和数据\n");
end

fprintf("增强演示完成。\n");