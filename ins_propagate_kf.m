function [state_hist, P_hist, diagnostics] = ins_propagate_kf(acc_body, q, t, is_still, params, opts)
% INS propagate KF (body-frame acc_bias state)
% State (12): [pos(3)_nav; vel(3)_nav; acc_bias_body(3); gyro_bias_body(3)]
% - Uses trapezoidal integration and rotates prev/current acc using q(k-1) and q(k)
% - Acc bias is in body frame; gravity is removed in nav frame

if nargin < 6, opts = struct(); end

% ---------- defaults ----------
fld = @(s,f,d) (isfield(s,f) && ~isempty(s.(f)))*1; %#ok<NASGU>
if ~isfield(opts,'applyPosCorrection'), opts.applyPosCorrection = false; end
if ~isfield(opts,'quick_bias_corr'), opts.quick_bias_corr = false; end
if ~isfield(opts,'bias_corr_alpha'), opts.bias_corr_alpha = 0.02; end
if ~isfield(opts,'cross_cov_scale'), opts.cross_cov_scale = 0.05; end
if ~isfield(opts,'still_window_samples'), opts.still_window_samples = []; end
if ~isfield(opts,'debug'), opts.debug = false; end
if ~isfield(opts,'init_bias_scale'), opts.init_bias_scale = 0.0; end
if ~isfield(opts,'VEL_INNOV_GATE'), opts.VEL_INNOV_GATE = 1.0; end
if ~isfield(opts,'MAX_DX_VEL'), opts.MAX_DX_VEL = 2.0; end
if ~isfield(opts,'MAX_DX_POS'), opts.MAX_DX_POS = 2.0; end
if ~isfield(opts,'MAX_DX_BIAS'), opts.MAX_DX_BIAS = 2.5; end
if ~isfield(opts,'bias_meas_R'), opts.bias_meas_R = 0.05; end
if ~isfield(opts,'TRUST_BIAS_MEAS_ANGLE_DEG'), opts.TRUST_BIAS_MEAS_ANGLE_DEG = 15.0; end
if ~isfield(opts,'BIAS_WINDOW_MAX_STD'), opts.BIAS_WINDOW_MAX_STD = 1.5; end
if ~isfield(opts,'init_bias'), opts.init_bias = [0;0;0]; end

% ---------- prepare ----------
N = length(t);
state = zeros(12,1);
state(7:9) = opts.init_bias(:);  % init body-frame acc bias
state_hist = zeros(N,12);
P_hist = zeros(12,12,N);

Ppos0 = 100; Pvel0 = 1.0; Paccb0 = 1.0; Pgyrob0 = 1e-2;
if isfield(opts,'init_bias_P_scale') && ~isempty(opts.init_bias_P_scale)
    Paccb0 = Paccb0 * opts.init_bias_P_scale;
end
P = diag([Ppos0,Ppos0,Ppos0, Pvel0,Pvel0,Pvel0, Paccb0,Paccb0,Paccb0, Pgyrob0,Pgyrob0,Pgyrob0]);
if isfield(opts,'P0') && ~isempty(opts.P0), P = opts.P0; end

state_hist(1,:) = state';
P_hist(:,:,1) = P;

dt_nom = median(diff(t));
if isempty(opts.still_window_samples), opts.still_window_samples = max(1, round(0.1/dt_nom)); end

% defaults in params
if ~isfield(params,'Q_vel'), params.Q_vel = 1e-4; end
if ~isfield(params,'Q_acc_bias'), params.Q_acc_bias = (0.05)^2; end
if ~isfield(params,'Q_gyro_bias'), params.Q_gyro_bias = (0.005*pi/180)^2; end
if ~isfield(params,'R_zupt_acc'), params.R_zupt_acc = 0.5; end
if ~isfield(params,'g'), params.g = 9.80665; end

MAX_P_DIAG = 1e6;

% diagnostics
diagnostics = struct();
diagnostics.mean_acc_still_list = [];
diagnostics.mean_vel_still_list = [];
diagnostics.still_times = [];
diagnostics.zupt_scaled_count = 0;
diagnostics.skipped_zupt = 0;
diagnostics.quick_bias_applied = 0;
diagnostics.innov_list = [];
diagnostics.bias_applied_count = 0;

% main loop
for k = 2:N
    dt = t(k) - t(k-1);
    a_prev_raw_body = acc_body(k-1,:)';
    a_curr_raw_body = acc_body(k,:)';
    a_bias_body = state(7:9);
    a_prev_corr_body = a_prev_raw_body - a_bias_body;
    a_curr_corr_body = a_curr_raw_body - a_bias_body;

    % rotate prev with q(k-1) and current with q(k)
    Cbn_prev = quat2dcm(q(k-1,:));
    Cbn_curr = quat2dcm(q(k,:));
    a_prev_nav = Cbn_prev * a_prev_corr_body - [0;0;params.g];
    a_curr_nav = Cbn_curr * a_curr_corr_body - [0;0;params.g];

    % propagate state (trapezoidal)
    vel = state(4:6) + 0.5*(a_prev_nav + a_curr_nav)*dt;
    pos = state(1:3) + 0.5*(state(4:6) + vel)*dt;
    state(1:3) = pos;
    state(4:6) = vel;

    % linearized F and Q (use current attitude for Jacobian)
    F = eye(12);
    F(1:3,4:6) = eye(3)*dt;
    F(4:6,7:9) = -Cbn_curr * dt;          % d vel / d acc_bias_body
    Q = zeros(12);
    Q(4:6,4:6) = eye(3) * params.Q_vel * dt;
    Q(7:9,7:9) = eye(3) * params.Q_acc_bias * dt;
    Q(10:12,10:12) = eye(3) * params.Q_gyro_bias * dt;
    P = F * P * F' + Q;

    % ZUPT update
    if is_still(k)
        w = opts.still_window_samples;
        idx0 = max(1, k-w+1);

        % velocity measurement v=0
        H = zeros(3,12); H(:,4:6) = eye(3);
        R = eye(3) * params.R_zupt_acc;
        y = - state(4:6);
        normy = norm(y);
        diagnostics.innov_list = [diagnostics.innov_list; normy];
        if normy > opts.VEL_INNOV_GATE
            scaleR = 1 + (normy / max(opts.VEL_INNOV_GATE,1e-3))^2;
            R_eff = R * scaleR;
            diagnostics.zupt_scaled_count = diagnostics.zupt_scaled_count + 1;
            if opts.debug
                fprintf('ZUPT scaled @ t=%.3f: |innov|=%.3f scaleR=%.2f\n', t(k), normy, scaleR);
            end
        else
            R_eff = R;
        end

        % reduce pos-vel cross covariance
        P(1:3,4:6) = P(1:3,4:6) * opts.cross_cov_scale;
        P(4:6,1:3) = P(4:6,1:3) * opts.cross_cov_scale;

        S = H * P * H' + R_eff;
        K = P * H' / S;
        dx = K * y;

        if ~opts.applyPosCorrection, dx(1:3) = 0; end
        if norm(dx(4:6)) > opts.MAX_DX_VEL, dx(4:6) = dx(4:6)*(opts.MAX_DX_VEL / norm(dx(4:6))); end
        if norm(dx(7:9)) > opts.MAX_DX_BIAS, dx(7:9) = dx(7:9)*(opts.MAX_DX_BIAS / norm(dx(7:9))); end

        state = state + dx;
        P = (eye(12) - K*H) * P;

        % Conservative bias measurement in still window
        window_body = acc_body(idx0:k,:);
        mean_body = mean(window_body,1)';
        Csum = zeros(3,3);
        for ii = idx0:k, Csum = Csum + quat2dcm(q(ii,:)); end
        Cmean = Csum / (k - idx0 + 1);
        g_body_pred = Cmean' * [0;0;params.g];
        angle_deg = acosd( max(-1,min(1, dot(mean_body,g_body_pred) / ((norm(mean_body)+eps)*(norm(g_body_pred)+eps)) )) );
        window_std = std(window_body,0,1)';

        if angle_deg < opts.TRUST_BIAS_MEAS_ANGLE_DEG && max(window_std) < opts.BIAS_WINDOW_MAX_STD
            acc_bias_desired = mean_body - g_body_pred;   % body frame
            H_b = zeros(3,12); H_b(:,7:9) = eye(3);
            R_b = eye(3) * opts.bias_meas_R;
            y_b = acc_bias_desired - state(7:9);
            S_b = H_b * P * H_b' + R_b;
            K_b = P * H_b' / S_b;
            dx_b = K_b * y_b;
            if norm(dx_b(7:9)) > opts.MAX_DX_BIAS
                dx_b(7:9) = dx_b(7:9) * (opts.MAX_DX_BIAS / norm(dx_b(7:9)));
            end
            state = state + dx_b;
            P = (eye(12) - K_b*H_b) * P;
            diagnostics.bias_applied_count = diagnostics.bias_applied_count + 1;
            if opts.debug
                fprintf('bias-meas applied @ t=%.3f angle=%.2f std=%.3f delta=[%.3f %.3f %.3f]\n', ...
                    t(k), angle_deg, max(window_std), dx_b(7), dx_b(8), dx_b(9));
            end
        else
            diagnostics.skipped_zupt = diagnostics.skipped_zupt + 1;
            if opts.debug
                fprintf('bias-meas skipped @ t=%.3f angle=%.2f std=%.3f\n', t(k), angle_deg, max(window_std));
            end
        end

        % store diagnostics
        diagnostics.mean_vel_still_list = [diagnostics.mean_vel_still_list; state(4:6)'];
        % mean nav accel residual for info
        mean_a_nav = zeros(3,1);
        for ii = idx0:k
            Cbn_i = quat2dcm(q(ii,:));
            mean_a_nav = mean_a_nav + (Cbn_i*(acc_body(ii,:)' - state(7:9)) - [0;0;params.g]);
        end
        mean_a_nav = mean_a_nav / (k - idx0 + 1);
        diagnostics.mean_acc_still_list = [diagnostics.mean_acc_still_list; mean_a_nav'];
        diagnostics.still_times = [diagnostics.still_times; t(k)];
    end

    % keep P symmetric and bounded
    P = 0.5*(P + P');
    d = diag(P);
    d = min(max(d, 1e-9), MAX_P_DIAG);
    for ii=1:12, P(ii,ii) = d(ii); end

    state_hist(k,:) = state';
    P_hist(:,:,k) = P;
end

if opts.debug
    fprintf('ins_propagate_kf: final acc_bias(body) = [%.4f %.4f %.4f]\n', state(7:9));
    if ~isempty(diagnostics.mean_acc_still_list)
        mm = mean(diagnostics.mean_acc_still_list,1);
        fprintf('mean(mean_a_nav over ZUPTs) = [%.3f %.3f %.3f]\n', mm(1), mm(2), mm(3));
    end
    fprintf('ZUPT counts: scaled=%d skipped=%d bias_applied=%d quick_bias_applied=%d\n', ...
        diagnostics.zupt_scaled_count, diagnostics.skipped_zupt, diagnostics.bias_applied_count, diagnostics.quick_bias_applied);
    if ~isempty(diagnostics.innov_list)
        fprintf('innov norm stats: mean=%.3f med=%.3f max=%.3f\n', mean(diagnostics.innov_list), median(diagnostics.innov_list), max(diagnostics.innov_list));
    end
end

end

