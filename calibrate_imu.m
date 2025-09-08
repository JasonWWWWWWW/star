function [ax_c, ay_c, az_c, gx_c, gy_c, gz_c, calib] = calibrate_imu(ax,ay,az,gx,gy,gz, t)
% calibrate_imu - minimal, robust bias calibration
% - Gyro: subtract average over initial 1 s (or first 100 samples)
% - Accel: leave as-is (bias to be estimated in KF), but report stats

N = numel(t);
dt_med = median(diff(t));
useN = min(N, max(10, round(1/max(dt_med,1e-3))));

% Gyro bias from first still window
gyro_bias = [mean(gx(1:useN)), mean(gy(1:useN)), mean(gz(1:useN))];
gx_c = gx - gyro_bias(1);
gy_c = gy - gyro_bias(2);
gz_c = gz - gyro_bias(3);

% Accel: pass-through (KF will estimate body-frame bias)
ax_c = ax; ay_c = ay; az_c = az;

calib = struct();
calib.gyro_bias = gyro_bias;
calib.acc_bias  = [0 0 0];
calib.acc_std   = [std(ax(1:useN)) std(ay(1:useN)) std(az(1:useN))];
calib.scale_factor = 1.0;

fprintf('Gyro bias from first %.3f s: [%.5f %.5f %.5f]\n', useN*dt_med, gyro_bias);
end

