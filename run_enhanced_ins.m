function run_enhanced_ins(dataPath, scenario)
% run_enhanced_ins - Simplified interface for enhanced INS
% 
% Usage:
%   run_enhanced_ins('data.csv')                    % Use default config
%   run_enhanced_ins('data.csv', 'high_noise')     % Use high_noise config
%   run_enhanced_ins('data.csv', 'slow_motion')    % Use slow_motion config
%
% Available scenarios:
%   'default', 'high_noise', 'low_noise', 'slow_motion', 'fast_motion',
%   'short_test', 'long_test', 'phone_horizontal', 'phone_vertical', 'debug'

if nargin < 1
    error('请提供数据文件路径');
end

if nargin < 2
    scenario = 'default';
end

fprintf('=== 增强型哑铃惯性导航系统 ===\n');
fprintf('数据文件: %s\n', dataPath);
fprintf('配置场景: %s\n', scenario);
fprintf('=====================================\n\n');

% Load configuration
config = config_enhanced_ins(scenario);

% Check if data file exists
if ~isfile(dataPath)
    error("找不到数据文件：%s，请确认路径。", dataPath);
end

% Read and validate data
fprintf('读取数据文件...\n');
D = readmatrix(dataPath);
if size(D,2) < 7
    error("CSV 格式应为 [t, ax, ay, az, gx, gy, gz]");
end

t = D(:,1);
N = length(t);
duration = t(end) - t(1);
fs_est = (N-1) / duration;

fprintf('数据统计:\n');
fprintf('  样本数: %d\n', N);
fprintf('  持续时间: %.2f s\n', duration);
fprintf('  估计采样率: %.1f Hz\n\n', fs_est);

% Preprocess data
fprintf('预处理IMU数据...\n');
[ax, ay, az, gx, gy, gz] = preprocess_imu_data(D);

% Enhanced calibration
fprintf('执行增强IMU标定...\n');
[ax_c, ay_c, az_c, gx_c, gy_c, gz_c, calib] = calibrate_imu_enhanced(ax,ay,az,gx,gy,gz, t);
fprintf("标定完成：\n");
fprintf("  加速度偏置: [%.4f %.4f %.4f] m/s²\n", calib.acc_bias);
fprintf("  陀螺偏置: [%.4f %.4f %.4f] deg/s\n", calib.gyro_bias*180/pi);
if isfield(calib, 'acc_scale')
    fprintf("  加速度尺度: [%.4f %.4f %.4f]\n", calib.acc_scale);
end

% Enhanced attitude estimation
fprintf('估计姿态...\n');
q = estimate_attitude_enhanced(ax_c, ay_c, az_c, gx_c, gy_c, gz_c, t, config.madgwick_beta);

% Enhanced ZUPT detection
fprintf('执行ZUPT检测...\n');
is_still = zupt_detect_enhanced([ax_c ay_c az_c], [gx_c gy_c gz_c], ...
                               median(diff(t)), config.zupt_opts);

% Enhanced initial bias estimation
fprintf('估计初始偏置...\n');
init_bias_body = estimate_initial_bias_enhanced(ax_c, ay_c, az_c, q, is_still, config.params);
config.opts_kf.init_bias = init_bias_body;

% Enhanced INS with Kalman Filter
fprintf('执行增强INS卡尔曼滤波...\n');
acc_body = [ax_c ay_c az_c];
[state_hist, P_hist, diagnostics] = ins_propagate_kf_enhanced(acc_body, q, t, is_still, ...
                                                             config.params, config.opts_kf);

% Results analysis and plotting
fprintf('生成结果分析...\n');
if config.enable_plots
    plot_results_enhanced(t, q, state_hist, is_still, diagnostics);
end

% Print comprehensive diagnostics
print_enhanced_diagnostics(diagnostics, t, state_hist);

% Performance summary
fprintf('\n=== 最终性能总结 ===\n');
final_pos = state_hist(end,1:3);
final_vel = state_hist(end,4:6);
max_pos_drift = max(abs(final_pos));
max_vel_drift = max(abs(final_vel));

fprintf("最终位置: [%.2f %.2f %.2f] m\n", final_pos);
fprintf("最终速度: [%.2f %.2f %.2f] m/s\n", final_vel);
fprintf("最大位置漂移: %.2f m\n", max_pos_drift);
fprintf("最大速度漂移: %.2f m/s\n", max_vel_drift);

% Performance rating
if max_pos_drift < 10 && max_vel_drift < 1
    rating = '优秀 ✓';
    color = '绿色';
elseif max_pos_drift < 30 && max_vel_drift < 3
    rating = '良好 ⚠';
    color = '黄色';
elseif max_pos_drift < 100 && max_vel_drift < 10
    rating = '一般 ⚠';
    color = '橙色';
else
    rating = '较差 ✗';
    color = '红色';
end

fprintf("总体评级: %s (%s)\n", rating, color);

% Save results if requested
if config.save_results
    save_results_to_file(state_hist, diagnostics, config, scenario);
end

fprintf('\n增强INS处理完成！\n');

% Recommendations based on performance
provide_recommendations(diagnostics, max_pos_drift, max_vel_drift, config);

end

function save_results_to_file(state_hist, diagnostics, config, scenario)
% Save results to MAT file

timestamp = datestr(now, 'yyyymmdd_HHMMSS');
filename = sprintf('enhanced_ins_results_%s_%s.mat', scenario, timestamp);

save(filename, 'state_hist', 'diagnostics', 'config', 'scenario');
fprintf('结果已保存到: %s\n', filename);

end

function provide_recommendations(diagnostics, pos_drift, vel_drift, config)
% Provide specific recommendations based on performance

fprintf('\n=== 改进建议 ===\n');

% High position drift
if pos_drift > 50
    fprintf('位置漂移过大建议:\n');
    fprintf('  - 检查初始静止时间是否足够 (建议>2秒)\n');
    fprintf('  - 尝试 ''slow_motion'' 配置\n');
    fprintf('  - 检查ZUPT检测质量\n');
end

% High velocity drift
if vel_drift > 3
    fprintf('速度漂移过大建议:\n');
    fprintf('  - 启用漂移补偿: opts_kf.drift_compensation = true\n');
    fprintf('  - 减小速度过程噪声: params.Q_vel\n');
    fprintf('  - 增强速度阻尼: opts_kf.velocity_damping = 0.95\n');
end

% Poor ZUPT performance
if diagnostics.zupt_scaled_count > length(diagnostics.innov_list) * 0.5
    fprintf('ZUPT性能差建议:\n');
    fprintf('  - 放宽创新门限: opts_kf.VEL_INNOV_GATE = 3.0\n');
    fprintf('  - 调整ZUPT检测参数: zupt_opts.acc_factor, zupt_opts.gyro_factor\n');
    fprintf('  - 尝试 ''high_noise'' 配置\n');
end

% Low bias correction rate
if diagnostics.bias_applied_count < diagnostics.zupt_scaled_count * 0.2
    fprintf('偏置修正不足建议:\n');
    fprintf('  - 放宽偏置测量条件: opts_kf.TRUST_BIAS_MEAS_ANGLE_DEG = 25\n');
    fprintf('  - 减小偏置测量噪声: opts_kf.bias_meas_R = 0.01\n');
    fprintf('  - 增加快速偏置修正: opts_kf.bias_corr_alpha = 0.15\n');
end

% Data quality issues
dt_nom = 0.01; % Assume typical sampling
still_ratio = 0;
if isfield(diagnostics, 'still_times')
    still_ratio = length(diagnostics.still_times) / 1000; % Rough estimate
end

if still_ratio < 0.1
    fprintf('静止时间不足建议:\n');
    fprintf('  - 增加更多静止段\n');
    fprintf('  - 放宽ZUPT检测: zupt_opts.acc_factor = 2\n');
    fprintf('  - 减小最小静止时间: zupt_opts.min_still_time = 0.5\n');
end

fprintf('=================\n\n');

% Suggest alternative configurations
if pos_drift > 30 || vel_drift > 2
    fprintf('建议尝试的配置:\n');
    if pos_drift > 100
        fprintf('  - run_enhanced_ins(''%s'', ''slow_motion'')\n', 'your_data.csv');
        fprintf('  - run_enhanced_ins(''%s'', ''high_noise'')\n', 'your_data.csv');
    else
        fprintf('  - run_enhanced_ins(''%s'', ''debug'') -- 查看详细信息\n', 'your_data.csv');
    end
    fprintf('\n');
end

end