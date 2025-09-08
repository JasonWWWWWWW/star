function [ax_c, ay_c, az_c, gx_c, gy_c, gz_c, calib] = calibrate_imu(ax,ay,az,gx,gy,gz, t)
% calibrate_imu - 简单静止段偏置标定（前 ~1 秒）
N = length(t);
useN = min(N, round(1 / median(diff(t))));
if useN < 10
    useN = min(N,10);
end

% 初步偏置估计
acc_bias = [mean(ax(1:useN)) mean(ay(1:useN)) mean(az(1:useN))];
gyro_bias = [mean(gx(1:useN)) mean(gy(1:useN)) mean(gz(1:useN))];

% 仅去除加计偏置（保持尺度为 1，避免过拟合）
[acc_c, acc_bias, acc_scale] = calibrate_accel(ax, ay, az);
ax_c = acc_c(:,1); ay_c = acc_c(:,2); az_c = acc_c(:,3);

gx_c = gx - gyro_bias(1);
gy_c = gy - gyro_bias(2);
gz_c = gz - gyro_bias(3);

% 输出标定信息
calib.acc_bias = acc_bias;
calib.gyro_bias = gyro_bias;
calib.acc_std  = [std(ax(1:useN)) std(ay(1:useN)) std(az(1:useN))];
calib.scale_factor = 1.0; %#ok<NASGU>
end

