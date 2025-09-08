function config = config_enhanced_ins(scenario)
% config_enhanced_ins - Configuration file for enhanced INS
% Returns optimized parameters for different scenarios
%
% Usage:
%   config = config_enhanced_ins('default')  % Default parameters
%   config = config_enhanced_ins('high_noise') % For noisy data
%   config = config_enhanced_ins('slow_motion') % For slow movements
%   config = config_enhanced_ins('fast_motion') % For fast movements
%   config = config_enhanced_ins('short_test') % For short test data
%   config = config_enhanced_ins('long_test')  % For long test data

if nargin < 1
    scenario = 'default';
end

% Base configuration
config = get_base_config();

% Scenario-specific adjustments
switch lower(scenario)
    case 'default'
        % Already set in base config
        
    case 'high_noise'
        % For noisy IMU data
        config.params.R_zupt_acc = 1.0;        % Higher observation noise
        config.params.Q_vel = 1e-4;            % Higher process noise
        config.zupt_opts.acc_factor = 5;       % More conservative ZUPT
        config.zupt_opts.gyro_factor = 5;
        config.zupt_opts.window = 0.5;         % Longer smoothing window
        config.opts_kf.VEL_INNOV_GATE = 3.0;   % More permissive gate
        config.opts_kf.bias_meas_R = 0.05;     % Less trust in bias measurements
        
    case 'low_noise'
        % For clean IMU data
        config.params.R_zupt_acc = 0.2;        % Lower observation noise
        config.params.Q_vel = 2e-5;            % Lower process noise
        config.zupt_opts.acc_factor = 2;       % More sensitive ZUPT
        config.zupt_opts.gyro_factor = 2;
        config.zupt_opts.window = 0.2;         % Shorter smoothing window
        config.opts_kf.VEL_INNOV_GATE = 2.0;   % Stricter gate
        config.opts_kf.bias_meas_R = 0.01;     % More trust in bias measurements
        
    case 'slow_motion'
        % For slow dumbbell movements
        config.params.Q_vel = 1e-5;            % Very low velocity noise
        config.params.Q_acc_bias = (0.01)^2;   % Stable bias
        config.zupt_opts.min_still_time = 1.2; % Longer still periods
        config.zupt_opts.acc_factor = 2.5;     % More sensitive
        config.opts_kf.velocity_damping = 0.95; % Stronger damping
        config.opts_kf.drift_compensation = true;
        config.madgwick_beta = 0.4;            % Lower beta for stability
        
    case 'fast_motion'
        % For fast dumbbell movements
        config.params.Q_vel = 2e-4;            % Higher velocity noise
        config.params.Q_acc_bias = (0.05)^2;   % More dynamic bias
        config.zupt_opts.min_still_time = 0.5; % Shorter still periods
        config.zupt_opts.acc_factor = 4;       % More conservative
        config.opts_kf.velocity_damping = 0.99; % Weaker damping
        config.opts_kf.MAX_DX_VEL = 2.0;       # Larger velocity updates
        config.madgwick_beta = 1.0;            % Higher beta for responsiveness
        
    case 'short_test'
        % For short duration tests (< 30 seconds)
        config.params.Q_acc_bias = (0.005)^2;  % Very stable bias
        config.zupt_opts.min_still_time = 0.3; % Shorter requirements
        config.zupt_opts.window = 0.2;         % Faster response
        config.opts_kf.init_bias_P_scale = 0.005; % Very confident in initial bias
        config.opts_kf.bias_corr_alpha = 0.15; % Faster bias correction
        
    case 'long_test'
        % For long duration tests (> 5 minutes)
        config.params.Q_acc_bias = (0.05)^2;   % Allow bias drift
        config.params.Q_gyro_bias = (0.01*pi/180)^2; % Allow gyro bias drift
        config.zupt_opts.min_still_time = 1.0; % Ensure quality ZUPT
        config.opts_kf.drift_compensation = true; % Essential for long tests
        config.opts_kf.velocity_damping = 0.995; % Gentle damping
        
    case 'phone_horizontal'
        % Phone held horizontally (screen up)
        config.coordinate_mapping = 'horizontal';
        config.initial_attitude_mode = 'horizontal';
        
    case 'phone_vertical'
        % Phone held vertically (screen facing user)
        config.coordinate_mapping = 'vertical';
        config.initial_attitude_mode = 'vertical';
        
    case 'debug'
        % For debugging and development
        config.opts_kf.debug = true;
        config.zupt_opts.debug = true;
        config.enable_detailed_plots = true;
        config.save_intermediate_results = true;
        
    otherwise
        warning('Unknown scenario: %s. Using default configuration.', scenario);
end

% Print configuration summary
print_config_summary(config, scenario);

end

function config = get_base_config()
% Base configuration with reasonable defaults

deg2rad = pi/180;

% KF parameters
config.params.g = 9.80665;
config.params.Q_gyro_bias = (0.002*deg2rad)^2;
config.params.Q_acc_bias  = (0.02)^2;
config.params.Q_vel       = 5e-5;
config.params.R_zupt_acc  = 0.5;

% Madgwick parameters
config.madgwick_beta = 0.6;

% ZUPT detection options
config.zupt_opts.window = 0.3;
config.zupt_opts.min_still_time = 0.8;
config.zupt_opts.acc_factor = 3;
config.zupt_opts.gyro_factor = 3;
config.zupt_opts.use_percentile = true;
config.zupt_opts.percentile_level = 15;
config.zupt_opts.gravity_check = true;
config.zupt_opts.consistency_check = true;

% KF options
config.opts_kf.debug = false;
config.opts_kf.quick_bias_corr = true;
config.opts_kf.bias_corr_alpha = 0.08;
config.opts_kf.cross_cov_scale = 0.02;
config.opts_kf.init_bias_scale = 1.0;
config.opts_kf.init_bias_P_scale = 0.01;
config.opts_kf.VEL_INNOV_GATE = 2.5;
config.opts_kf.MAX_DX_VEL = 1.5;
config.opts_kf.MAX_DX_POS = 1.0;
config.opts_kf.MAX_DX_BIAS = 1.0;
config.opts_kf.bias_meas_R = 0.02;
config.opts_kf.TRUST_BIAS_MEAS_ANGLE_DEG = 20.0;
config.opts_kf.BIAS_WINDOW_MAX_STD = 1.0;
config.opts_kf.drift_compensation = true;
config.opts_kf.velocity_damping = 0.98;

% Data preprocessing options
config.preprocess_opts.enable_filtering = true;
config.preprocess_opts.acc_cutoff_hz = 20;
config.preprocess_opts.gyro_cutoff_hz = 30;
config.preprocess_opts.remove_outliers = true;

% Coordinate system and initial attitude
config.coordinate_mapping = 'auto';  % 'auto', 'horizontal', 'vertical'
config.initial_attitude_mode = 'gravity_based';

% Plotting and output options
config.enable_plots = true;
config.enable_detailed_plots = false;
config.save_results = false;
config.save_intermediate_results = false;

end

function print_config_summary(config, scenario)
% Print a summary of the configuration

fprintf('\n========== 配置总结: %s ==========\n', upper(scenario));

fprintf('KF参数:\n');
fprintf('  重力: %.5f m/s²\n', config.params.g);
fprintf('  陀螺偏置噪声: %.2e (rad/s)²\n', config.params.Q_gyro_bias);
fprintf('  加速度偏置噪声: %.2e (m/s²)²\n', config.params.Q_acc_bias);
fprintf('  速度过程噪声: %.2e (m/s)²\n', config.params.Q_vel);
fprintf('  ZUPT观测噪声: %.2f (m/s)²\n', config.params.R_zupt_acc);

fprintf('\nZUPT检测参数:\n');
fprintf('  检测窗口: %.1f s\n', config.zupt_opts.window);
fprintf('  最小静止时间: %.1f s\n', config.zupt_opts.min_still_time);
fprintf('  加速度阈值因子: %.1f\n', config.zupt_opts.acc_factor);
fprintf('  陀螺阈值因子: %.1f\n', config.zupt_opts.gyro_factor);

fprintf('\n卡尔曼滤波选项:\n');
fprintf('  速度创新门限: %.1f\n', config.opts_kf.VEL_INNOV_GATE);
fprintf('  最大速度更新: %.1f m/s\n', config.opts_kf.MAX_DX_VEL);
fprintf('  最大偏置更新: %.1f m/s²\n', config.opts_kf.MAX_DX_BIAS);
fprintf('  偏置测量噪声: %.3f\n', config.opts_kf.bias_meas_R);
fprintf('  漂移补偿: %s\n', bool2str(config.opts_kf.drift_compensation));

fprintf('\nMadgwick参数:\n');
fprintf('  Beta参数: %.1f\n', config.madgwick_beta);

fprintf('=======================================\n\n');

end

function str = bool2str(val)
if val
    str = '启用';
else
    str = '禁用';
end
end