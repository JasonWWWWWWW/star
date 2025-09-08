function [state_hist, P_hist, diagnostics] = ins_propagate_kf_enhanced(acc_body, q, t, is_still, params, opts)
% ins_propagate_kf_enhanced - Enhanced INS with Kalman Filter
% Improved drift compensation and robust bias estimation

if nargin < 6, opts = struct(); end

% Enhanced default options
if ~isfield(opts,'applyPosCorrection'), opts.applyPosCorrection = false; end
if ~isfield(opts,'quick_bias_corr'), opts.quick_bias_corr = true; end
if ~isfield(opts,'bias_corr_alpha'), opts.bias_corr_alpha = 0.08; end
if ~isfield(opts,'cross_cov_scale'), opts.cross_cov_scale = 0.02; end
if ~isfield(opts,'still_window_samples'), opts.still_window_samples = []; end
if ~isfield(opts,'debug'), opts.debug = false; end
if ~isfield(opts,'init_bias_scale'), opts.init_bias_scale = 1.0; end
if ~isfield(opts,'VEL_INNOV_GATE'), opts.VEL_INNOV_GATE = 2.5; end
if ~isfield(opts,'MAX_DX_VEL'), opts.MAX_DX_VEL = 1.5; end
if ~isfield(opts,'MAX_DX_POS'), opts.MAX_DX_POS = 1.0; end
if ~isfield(opts,'MAX_DX_BIAS'), opts.MAX_DX_BIAS = 1.0; end
if ~isfield(opts,'bias_meas_R'), opts.bias_meas_R = 0.02; end
if ~isfield(opts,'TRUST_BIAS_MEAS_ANGLE_DEG'), opts.TRUST_BIAS_MEAS_ANGLE_DEG = 20.0; end
if ~isfield(opts,'BIAS_WINDOW_MAX_STD'), opts.BIAS_WINDOW_MAX_STD = 1.0; end
if ~isfield(opts,'drift_compensation'), opts.drift_compensation = true; end
if ~isfield(opts,'velocity_damping'), opts.velocity_damping = 0.98; end

N = length(t);
dt_nom = median(diff(t));

% Enhanced initial covariance
Ppos0 = 50; Pvel0 = 0.5; Paccb0 = 0.5; Pgyrob0 = 1e-3;
if isfield(opts,'init_bias_P_scale')
    Paccb0 = Paccb0 * opts.init_bias_P_scale;
end

P = diag([Ppos0,Ppos0,Ppos0, Pvel0,Pvel0,Pvel0, Paccb0,Paccb0,Paccb0, Pgyrob0,Pgyrob0,Pgyrob0]);
if isfield(opts,'P0') && ~isempty(opts.P0), P = opts.P0; end

% Initialize state
state = zeros(12,1);
if isfield(opts,'init_bias') && ~isempty(opts.init_bias)
    state(7:9) = opts.init_bias(:) * opts.init_bias_scale;
end

% Storage
state_hist = zeros(N,12);
P_hist = zeros(12,12,N);
state_hist(1,:) = state';
P_hist(:,:,1) = P;

% Enhanced process noise parameters
if ~isfield(params,'Q_vel'), params.Q_vel = 5e-5; end
if ~isfield(params,'Q_acc_bias'), params.Q_acc_bias = (0.02)^2; end
if ~isfield(params,'Q_gyro_bias'), params.Q_gyro_bias = (0.002*pi/180)^2; end
if ~isfield(params,'R_zupt_acc'), params.R_zupt_acc = 0.5; end
if ~isfield(params,'g'), params.g = 9.80665; end

% Enhanced diagnostics
diagnostics = struct();
diagnostics.mean_acc_still_list = [];
diagnostics.mean_vel_still_list = [];
diagnostics.still_times = [];
diagnostics.zupt_scaled_count = 0;
diagnostics.skipped_zupt = 0;
diagnostics.quick_bias_applied = 0;
diagnostics.innov_list = [];
diagnostics.bias_applied_count = 0;
diagnostics.drift_corrections = 0;
diagnostics.velocity_resets = 0;

% Window size for ZUPT
if isempty(opts.still_window_samples)
    opts.still_window_samples = max(5, round(0.2/dt_nom));
end

% Previous acceleration for integration
a_prev_raw_body = acc_body(1,:)';
MAX_P_DIAG = 1e6;

% Enhanced motion detection
motion_detector = create_motion_detector(dt_nom);

% Main propagation loop
for k = 2:N
    dt = t(k) - t(k-1);
    a_curr_raw_body = acc_body(k,:)';
    
    % Current bias estimate
    a_bias_body = state(7:9);
    
    % Bias-corrected accelerations
    a_prev_corr_body = a_prev_raw_body - a_bias_body;
    a_curr_corr_body = a_curr_raw_body - a_bias_body;
    
    % Rotate to navigation frame
    Cbn = quat2dcm(q(k,:));
    a_prev_nav = Cbn * a_prev_corr_body - [0;0;params.g];
    a_curr_nav = Cbn * a_curr_corr_body - [0;0;params.g];
    
    % Enhanced motion detection
    motion_level = update_motion_detector(motion_detector, a_curr_nav, norm([a_curr_raw_body; acc_body(k,:)']));
    
    % State propagation with trapezoidal integration
    vel_new = state(4:6) + 0.5*(a_prev_nav + a_curr_nav)*dt;
    pos_new = state(1:3) + 0.5*(state(4:6) + vel_new)*dt;
    
    % Apply velocity damping during motion (optional drift compensation)
    if opts.drift_compensation && motion_level < 0.3 && ~is_still(k)
        vel_new = vel_new * opts.velocity_damping;
        diagnostics.drift_corrections = diagnostics.drift_corrections + 1;
    end
    
    state(1:3) = pos_new;
    state(4:6) = vel_new;
    
    % Enhanced covariance propagation
    F = eye(12);
    F(1:3,4:6) = eye(3)*dt;
    F(4:6,7:9) = -Cbn * dt;
    
    % Adaptive process noise based on motion level
    Q_vel_adaptive = params.Q_vel * (1 + 2*motion_level);
    Q_acc_bias_adaptive = params.Q_acc_bias * (1 + motion_level);
    
    Q = zeros(12);
    Q(4:6,4:6) = eye(3) * Q_vel_adaptive * dt;
    Q(7:9,7:9) = eye(3) * Q_acc_bias_adaptive * dt;
    Q(10:12,10:12) = eye(3) * params.Q_gyro_bias * dt;
    
    P = F * P * F' + Q;
    
    % ZUPT processing when still
    if is_still(k)
        [state, P, zupt_diagnostics] = process_zupt_enhanced(state, P, acc_body, q, k, t, opts, params, diagnostics);
        
        % Update diagnostics
        diagnostics.zupt_scaled_count = diagnostics.zupt_scaled_count + zupt_diagnostics.scaled;
        diagnostics.skipped_zupt = diagnostics.skipped_zupt + zupt_diagnostics.skipped;
        diagnostics.quick_bias_applied = diagnostics.quick_bias_applied + zupt_diagnostics.quick_bias;
        diagnostics.bias_applied_count = diagnostics.bias_applied_count + zupt_diagnostics.bias_applied;
        diagnostics.innov_list = [diagnostics.innov_list; zupt_diagnostics.innov_norm];
        
        if ~isempty(zupt_diagnostics.mean_acc_nav)
            diagnostics.mean_acc_still_list = [diagnostics.mean_acc_still_list; zupt_diagnostics.mean_acc_nav'];
            diagnostics.mean_vel_still_list = [diagnostics.mean_vel_still_list; state(4:6)'];
            diagnostics.still_times = [diagnostics.still_times; t(k)];
        end
    end
    
    % Velocity reset for extreme drift
    if norm(state(4:6)) > 10 && motion_level < 0.2
        state(4:6) = state(4:6) * 0.1;  % Aggressive velocity reset
        diagnostics.velocity_resets = diagnostics.velocity_resets + 1;
        if opts.debug
            fprintf('执行速度重置 @ t=%.3f, |v|=%.2f\n', t(k), norm(state(4:6)));
        end
    end
    
    % Maintain covariance properties
    P = 0.5*(P + P');
    d = diag(P);
    d = min(max(d, 1e-9), MAX_P_DIAG);
    for ii=1:12, P(ii,ii) = d(ii); end
    
    % Store results
    state_hist(k,:) = state';
    P_hist(:,:,k) = P;
    a_prev_raw_body = a_curr_raw_body;
end

% Final diagnostics
if opts.debug
    fprintf('ins_propagate_kf_enhanced: 最终加速度偏置(body) = [%.4f %.4f %.4f]\n', state(7:9));
    if ~isempty(diagnostics.mean_acc_still_list)
        mm = mean(diagnostics.mean_acc_still_list,1);
        fprintf('ZUPT期间平均导航系加速度 = [%.3f %.3f %.3f]\n', mm);
    end
    fprintf('增强KF统计: scaled=%d skipped=%d bias_applied=%d quick_bias=%d drift_corr=%d vel_reset=%d\n', ...
        diagnostics.zupt_scaled_count, diagnostics.skipped_zupt, diagnostics.bias_applied_count, ...
        diagnostics.quick_bias_applied, diagnostics.drift_corrections, diagnostics.velocity_resets);
end

end

% ========== Helper Functions ==========

function motion_detector = create_motion_detector(dt)
motion_detector.window_size = max(10, round(0.5/dt));
motion_detector.acc_history = [];
motion_detector.gyro_history = [];
end

function motion_level = update_motion_detector(detector, acc_nav, gyro_norm)
% Returns motion level between 0 (static) and 1 (high motion)
persistent acc_buffer gyro_buffer

if isempty(acc_buffer)
    acc_buffer = zeros(detector.window_size, 3);
    gyro_buffer = zeros(detector.window_size, 1);
end

% Shift buffer and add new data
acc_buffer = [acc_buffer(2:end,:); acc_nav'];
gyro_buffer = [gyro_buffer(2:end); gyro_norm];

% Compute motion indicators
acc_var = mean(var(acc_buffer));
gyro_var = var(gyro_buffer);

% Normalize to [0,1] range
motion_level = min(1, sqrt(acc_var)/2 + sqrt(gyro_var)/0.5);
end

function [state_out, P_out, diagnostics] = process_zupt_enhanced(state, P, acc_body, q, k, t, opts, params, global_diag)
% Enhanced ZUPT processing with multiple correction mechanisms

diagnostics = struct('scaled', 0, 'skipped', 0, 'quick_bias', 0, 'bias_applied', 0, ...
                    'innov_norm', 0, 'mean_acc_nav', []);

state_out = state;
P_out = P;

% Build analysis window
w = opts.still_window_samples;
idx0 = max(1, k-w+1);
window_body = acc_body(idx0:k,:);

% Compute mean navigation acceleration over window
mean_a_nav = zeros(3,1);
for ii = idx0:k
    Cbn_i = quat2dcm(q(ii,:));
    mean_a_nav = mean_a_nav + Cbn_i * (acc_body(ii,:)' - state(7:9));
end
mean_a_nav = mean_a_nav / (k - idx0 + 1) - [0;0;params.g];

diagnostics.mean_acc_nav = mean_a_nav;

% Standard velocity ZUPT update
H = zeros(3,12); H(:,4:6) = eye(3);
R = eye(3) * params.R_zupt_acc;
y = -state(4:6);  % Innovation (measured=0, predicted=velocity)

diagnostics.innov_norm = norm(y);

% Adaptive observation noise scaling
if diagnostics.innov_norm > opts.VEL_INNOV_GATE
    scale_factor = 1 + (diagnostics.innov_norm / opts.VEL_INNOV_GATE)^2;
    R_scaled = R * scale_factor;
    diagnostics.scaled = 1;
    if opts.debug
        fprintf('ZUPT缩放 @ t=%.3f: |创新|=%.3f, 缩放=%.2f\n', t(k), diagnostics.innov_norm, scale_factor);
    end
else
    R_scaled = R;
end

% Reduce cross-covariance
P_out(1:3,4:6) = P_out(1:3,4:6) * opts.cross_cov_scale;
P_out(4:6,1:3) = P_out(4:6,1:3) * opts.cross_cov_scale;

% Kalman update
S = H * P_out * H' + R_scaled;
K = P_out * H' / S;
dx = K * y;

% Limit updates
if ~opts.applyPosCorrection, dx(1:3) = 0; end
if norm(dx(1:3)) > opts.MAX_DX_POS, dx(1:3) = dx(1:3)*(opts.MAX_DX_POS/norm(dx(1:3))); end
if norm(dx(4:6)) > opts.MAX_DX_VEL, dx(4:6) = dx(4:6)*(opts.MAX_DX_VEL/norm(dx(4:6))); end
if norm(dx(7:9)) > opts.MAX_DX_BIAS, dx(7:9) = dx(7:9)*(opts.MAX_DX_BIAS/norm(dx(7:9))); end

% Apply update
state_out = state_out + dx;
P_out = (eye(12) - K*H) * P_out;

% Enhanced bias correction mechanisms
if opts.quick_bias_corr
    window_std = std(window_body, 0, 1)';
    if norm(mean_a_nav) < 1.5 && max(window_std) < 0.8
        % Quick bias correction in navigation frame
        delta_nav = opts.bias_corr_alpha * mean_a_nav;
        Cbn_k = quat2dcm(q(k,:));
        delta_body = Cbn_k' * delta_nav;
        
        max_step = 0.3;
        if norm(delta_body) > max_step
            delta_body = delta_body * (max_step/norm(delta_body));
        end
        
        state_out(7:9) = state_out(7:9) + delta_body;
        diagnostics.quick_bias = 1;
    end
end

% Bias measurement update (using gravity alignment)
mean_body = mean(window_body, 1)';
Csum = zeros(3,3);
for ii = idx0:k
    Csum = Csum + quat2dcm(q(ii,:));
end
Cmean = Csum / (k - idx0 + 1);
g_body_pred = Cmean' * [0;0;params.g];

% Quality checks for bias measurement
cos_angle = dot(mean_body, g_body_pred) / (norm(mean_body)*norm(g_body_pred) + eps);
angle_deg = acosd(max(-1, min(1, cos_angle)));
window_std = std(window_body, 0, 1)';

if angle_deg < opts.TRUST_BIAS_MEAS_ANGLE_DEG && max(window_std) < opts.BIAS_WINDOW_MAX_STD
    acc_bias_desired = mean_body - g_body_pred;
    
    H_b = zeros(3,12); H_b(:,7:9) = eye(3);
    R_b = eye(3) * opts.bias_meas_R;
    y_b = acc_bias_desired;
    r_b = y_b - state_out(7:9);
    
    S_b = H_b * P_out * H_b' + R_b;
    K_b = P_out * H_b' / S_b;
    dx_b = K_b * r_b;
    
    if norm(dx_b(7:9)) > opts.MAX_DX_BIAS
        dx_b(7:9) = dx_b(7:9) * (opts.MAX_DX_BIAS / norm(dx_b(7:9)));
    end
    
    state_out = state_out + dx_b;
    P_out = (eye(12) - K_b*H_b) * P_out;
    diagnostics.bias_applied = 1;
end

end

function C = quat2dcm(q)
% Convert quaternion to DCM
w = q(1); x = q(2); y = q(3); z = q(4);
C = [1-2*(y^2+z^2),   2*(x*y - z*w),   2*(x*z + y*w);
     2*(x*y + z*w),   1-2*(x^2+z^2),   2*(y*z - x*w);
     2*(x*z - y*w),   2*(y*z + x*w),   1-2*(x^2+y^2)];
end