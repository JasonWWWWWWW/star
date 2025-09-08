% main_demo.m  IMU -> attitude -> INS+KF -> plots (fixed axis mapping)
clear; close all; clc;

% ------------ User config -------------
dataPath = "merged_sensor_data.csv";   % CSV columns: [t, ax, ay, az, gx, gy, gz]
gyro_is_deg = false;                    % set true if gyro in deg/s
beta = 0.8;                             % Madgwick parameter

% KF params (robust starting point)
params = struct();
params.g = 9.80665;
params.Q_gyro_bias = (0.005*pi/180)^2;
params.Q_acc_bias  = (0.05)^2;
params.Q_vel       = 1e-4;
params.R_zupt_acc  = 1.0;               % velocity measurement noise during ZUPT

% Optional axis/sign mapping matrices (body <- csv)
% Keep identity by default. If your device axes differ, edit here.
A_acc  = eye(3);
A_gyro = eye(3);

% ------------- Load data --------------
if ~isfile(dataPath)
    error("Data file not found: %s", dataPath);
end
D = readmatrix(dataPath);
if size(D,2) < 7
    error("CSV must be [t, ax, ay, az, gx, gy, gz]");
end

t = D(:,1);
acc_csv = [D(:,2), D(:,3), D(:,4)];     % DO NOT remap columns here
gyro_csv = [D(:,5), D(:,6), D(:,7)];     % DO NOT remap columns here

% Apply optional mapping and units
acc_raw  = (acc_csv * A_acc');
gyro_raw = (gyro_csv * A_gyro');
if gyro_is_deg
    gyro_raw = gyro_raw * (pi/180);
end

dt_vec = diff(t);
dt = median(dt_vec);
N = numel(t);

% ------------ Calibration -------------
[ax_c, ay_c, az_c, gx_c, gy_c, gz_c, calib] = calibrate_imu(acc_raw(:,1),acc_raw(:,2),acc_raw(:,3), ...
                                                            gyro_raw(:,1),gyro_raw(:,2),gyro_raw(:,3), t);
fprintf("Calibration done. Gyro bias = [%.5f %.5f %.5f]\n", calib.gyro_bias);

% ------------ Attitude (Madgwick) -------------
q = zeros(N,4);
useN = min(N, max(10, round(1/median(diff(t)))));
a0 = [mean(ax_c(1:useN)), mean(ay_c(1:useN)), mean(az_c(1:useN))];
roll0  = atan2( a0(2), a0(3) );
pitch0 = atan2( -a0(1), sqrt(a0(2)^2 + a0(3)^2) );
yaw0 = 0;
cy = cos(yaw0*0.5); sy = sin(yaw0*0.5);
cp = cos(pitch0*0.5); sp = sin(pitch0*0.5);
cr = cos(roll0*0.5); sr = sin(roll0*0.5);
q(1,:) = [cr*cp*cy + sr*sp*sy, ...
          sr*cp*cy - cr*sp*sy, ...
          cr*sp*cy + sr*cp*sy, ...
          cr*cp*sy - sr*sp*cy];

for k = 2:N
    dtk = t(k) - t(k-1);
    q(k,:) = madgwick_ahrs([ax_c(k) ay_c(k) az_c(k)], [gx_c(k) gy_c(k) gz_c(k)], q(k-1,:), beta, dtk);
    if mod(k,200)==0
        q(k,:) = q(k,:) / norm(q(k,:));
    end
end
for k=1:N
    q(k,:) = q(k,:) / max(norm(q(k,:)), 1e-12);
end

% ------------ Acc in nav frame (for plotting) -------------
acc_nav = zeros(N,3);
for k=1:N
    Cbn = quat2dcm(q(k,:));
    a_nav = Cbn * [ax_c(k); ay_c(k); az_c(k)];
    a_nav = a_nav - [0;0;params.g];
    acc_nav(k,:) = a_nav';
end

% ------------ ZUPT detection -------------
is_still = zupt_detect([ax_c ay_c az_c], [gx_c gy_c gz_c], dt);

% ------------ Initial body-frame accelerometer bias -------------
still_idx = find(is_still);
if ~isempty(still_idx)
    idxs = still_idx(1:min(numel(still_idx), max(10, round(0.5/dt))));
    mean_acc_body_still = mean([ax_c(idxs), ay_c(idxs), az_c(idxs)],1)';
    Csum = zeros(3,3);
    for ii = idxs'
        Csum = Csum + quat2dcm(q(ii,:));
    end
    Cmean = Csum / numel(idxs);
    g_body_pred = Cmean' * [0;0;params.g];
    init_bias_body = mean_acc_body_still - g_body_pred;
else
    init_bias_body = [0;0;0];
end
fprintf('Init acc bias (body) from first still window = [%.4f %.4f %.4f]\n', init_bias_body);

% ------------ Run KF (body-frame acc) -------------
opts_kf = struct();
opts_kf.debug = true;
opts_kf.quick_bias_corr = false;
opts_kf.bias_corr_alpha = 0.18;
opts_kf.cross_cov_scale = 0.05;
opts_kf.init_bias = init_bias_body;      % use directly
opts_kf.init_bias_P_scale = 0.1;         % smaller initial variance on acc bias
opts_kf.VEL_INNOV_GATE = 1.0;
opts_kf.MAX_DX_VEL = 2.0;
opts_kf.MAX_DX_BIAS = 2.5;
opts_kf.MAX_DX_POS = 2.0;
opts_kf.bias_meas_R = 0.05;

acc_body = [ax_c ay_c az_c];
[state_hist, P_hist, diagnostics] = ins_propagate_kf(acc_body, q, t, is_still, params, opts_kf);

% ------------ Plots and diagnostics -------------
plot_results(t, q, state_hist, acc_nav, is_still);
print_run_diagnostics(t, acc_body, acc_nav, is_still, state_hist, diagnostics);

fprintf("Done.\n");

