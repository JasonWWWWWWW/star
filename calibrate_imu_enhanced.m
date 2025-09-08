function [ax_c, ay_c, az_c, gx_c, gy_c, gz_c, calib] = calibrate_imu_enhanced(ax,ay,az,gx,gy,gz, t)
% calibrate_imu_enhanced - Enhanced IMU calibration with multiple static periods
% Uses multiple static periods and ellipsoid fitting for better calibration

N = length(t);
dt = median(diff(t));

% ---------- Enhanced Static Period Detection ----------
% Find multiple static periods for robust calibration
static_periods = find_static_periods(ax, ay, az, gx, gy, gz, dt);

if isempty(static_periods)
    % Fallback to simple initial period
    useN = min(N, max(50, round(2 / dt)));  % Use longer initial period
    static_periods = [1, useN];
end

fprintf('检测到 %d 个静止段用于标定\n', size(static_periods, 1));

% ---------- Enhanced Gyroscope Calibration ----------
gyro_bias = zeros(1,3);
for i = 1:size(static_periods, 1)
    period = static_periods(i,1):static_periods(i,2);
    period_bias = [mean(gx(period)), mean(gy(period)), mean(gz(period))];
    gyro_bias = gyro_bias + period_bias;
end
gyro_bias = gyro_bias / size(static_periods, 1);

% Apply gyroscope calibration
gx_c = gx - gyro_bias(1);
gy_c = gy - gyro_bias(2);
gz_c = gz - gyro_bias(3);

% ---------- Enhanced Accelerometer Calibration ----------
% Method 1: Multi-period bias estimation
acc_bias_simple = zeros(1,3);
for i = 1:size(static_periods, 1)
    period = static_periods(i,1):static_periods(i,2);
    period_bias = [mean(ax(period)), mean(ay(period)), mean(az(period))];
    acc_bias_simple = acc_bias_simple + period_bias;
end
acc_bias_simple = acc_bias_simple / size(static_periods, 1);

% Method 2: Gravity-based calibration
g = 9.80665;
acc_mean = acc_bias_simple;
acc_norm = norm(acc_mean);

% Estimate scale factors assuming the static measurement should equal gravity
if acc_norm > 5  % Reasonable gravity measurement
    % Simple uniform scaling
    scale_factor_uniform = g / acc_norm;
    acc_scale = [scale_factor_uniform, scale_factor_uniform, scale_factor_uniform];
    
    % Adjust bias to account for gravity direction
    % Assume the mean acceleration during static periods should point to gravity
    gravity_direction = acc_mean / acc_norm;  % Unit vector
    expected_acc = gravity_direction * g;
    acc_bias = acc_mean - expected_acc;
else
    % Fallback to simple bias-only calibration
    acc_bias = acc_bias_simple;
    acc_scale = [1, 1, 1];
end

% Method 3: Enhanced ellipsoid fitting (if enough data variation)
try
    if size(static_periods, 1) >= 3 || N > 1000
        [acc_bias_ellipsoid, acc_scale_ellipsoid] = calibrate_accel_ellipsoid([ax, ay, az]);
        
        % Use ellipsoid result if it's reasonable
        if all(acc_scale_ellipsoid > 0.5) && all(acc_scale_ellipsoid < 2.0)
            acc_bias = acc_bias_ellipsoid;
            acc_scale = acc_scale_ellipsoid;
            fprintf('使用椭球拟合标定结果\n');
        else
            fprintf('椭球拟合结果异常，使用重力基准标定\n');
        end
    end
catch
    fprintf('椭球拟合失败，使用重力基准标定\n');
end

% Apply accelerometer calibration
ax_c = (ax - acc_bias(1)) * acc_scale(1);
ay_c = (ay - acc_bias(2)) * acc_scale(2);
az_c = (az - acc_bias(3)) * acc_scale(3);

% ---------- Validation ----------
% Check calibration quality using static periods
total_error = 0;
for i = 1:size(static_periods, 1)
    period = static_periods(i,1):static_periods(i,2);
    acc_cal_mean = [mean(ax_c(period)), mean(ay_c(period)), mean(az_c(period))];
    gravity_error = abs(norm(acc_cal_mean) - g);
    total_error = total_error + gravity_error;
end
avg_gravity_error = total_error / size(static_periods, 1);

fprintf('标定质量检查: 平均重力误差 = %.4f m/s² (目标 < 0.5)\n', avg_gravity_error);

% ---------- Store Calibration Results ----------
calib.acc_bias = acc_bias;
calib.gyro_bias = gyro_bias;
calib.acc_scale = acc_scale;
calib.gravity_error = avg_gravity_error;
calib.static_periods = static_periods;
calib.n_static_periods = size(static_periods, 1);

end

function static_periods = find_static_periods(ax, ay, az, gx, gy, gz, dt)
% Find multiple static periods in the data
N = length(ax);
window = max(10, round(0.5 / dt));  % 0.5 second window
min_duration = max(20, round(1.0 / dt));  % Minimum 1 second duration

% Calculate moving statistics
acc_mag = sqrt(ax.^2 + ay.^2 + az.^2);
gyro_mag = sqrt(gx.^2 + gy.^2 + gz.^2);

acc_std = movstd(acc_mag, window);
gyro_std = movstd(gyro_mag, window);

% Adaptive thresholds
acc_threshold = prctile(acc_std, 20);  % 20th percentile
gyro_threshold = prctile(gyro_std, 20);

% Find static candidates
is_static = (acc_std < acc_threshold) & (gyro_std < gyro_threshold);

% Find continuous static periods
static_periods = [];
if any(is_static)
    % Find start and end of static periods
    d = diff([0; is_static; 0]);
    starts = find(d == 1);
    ends = find(d == -1) - 1;
    
    for i = 1:length(starts)
        duration = ends(i) - starts(i) + 1;
        if duration >= min_duration
            static_periods = [static_periods; starts(i), ends(i)];
        end
    end
end

% Limit to first few periods to avoid computational overhead
if size(static_periods, 1) > 5
    static_periods = static_periods(1:5, :);
end

end

function [bias, scale] = calibrate_accel_ellipsoid(acc_data)
% Simple ellipsoid fitting for accelerometer calibration
% This is a simplified version - for production use, consider more robust methods

try
    % Use only a subset of data to speed up computation
    N = size(acc_data, 1);
    if N > 2000
        idx = round(linspace(1, N, 2000));
        acc_data = acc_data(idx, :);
    end
    
    % Fit ellipsoid using least squares
    x = acc_data(:,1); y = acc_data(:,2); z = acc_data(:,3);
    
    % Set up normal equations for ellipsoid fitting
    % (x-bx)²/sx² + (y-by)²/sy² + (z-bz)²/sz² = r²
    % Simplified to: assume uniform scaling and estimate bias
    
    % Estimate bias as centroid
    bias = mean(acc_data, 1);
    
    % Estimate scale to make norm approximately equal to gravity
    acc_centered = acc_data - bias;
    norms = sqrt(sum(acc_centered.^2, 2));
    avg_norm = mean(norms);
    
    if avg_norm > 1  % Reasonable measurement
        scale_factor = 9.80665 / avg_norm;
        scale = [scale_factor, scale_factor, scale_factor];
    else
        scale = [1, 1, 1];
    end
    
catch
    % Fallback
    bias = mean(acc_data, 1);
    scale = [1, 1, 1];
end

end