function [acc_c, acc_bias, acc_scale] = calibrate_accel(ax, ay, az)
% 简单偏置标定：取前 1 秒均值作为偏置，仅去偏不做尺度
acc_raw = [ax(:), ay(:), az(:)];
Nall = numel(ax);
N = min( max(10, round(0.99*min(Nall, 1))), Nall );
if N <= 0, N = min(10, Nall); end
acc_bias = mean(acc_raw(1:N,:),1);
acc_c = acc_raw - acc_bias;
acc_scale = [1 1 1];
fprintf('Accel bias only calibration applied. Bias = [%g %g %g]\n', acc_bias);
end

