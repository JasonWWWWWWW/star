function [ax, ay, az, gx, gy, gz] = preprocess_imu_data(D)
% preprocess_imu_data - Enhanced IMU data preprocessing with filtering
% Applies coordinate transformation and low-pass filtering

% Extract raw data
ax_raw = D(:,2); ay_raw = D(:,3); az_raw = D(:,4);
gx_raw = D(:,5); gy_raw = D(:,6); gz_raw = D(:,7);

% Apply coordinate transformation (adjust based on your phone orientation)
% This mapping assumes the phone coordinate system needs transformation
ax_temp = -ay_raw;  % Phone Y -> Body X (negative)
ay_temp = -ax_raw;  % Phone X -> Body Y (negative)  
az_temp = -az_raw;  % Phone Z -> Body Z (negative)

gx_temp = -gy_raw;  % Phone Y -> Body X (negative)
gy_temp = -gx_raw;  % Phone X -> Body Y (negative)
gz_temp = -gz_raw;  % Phone Z -> Body Z (negative)

% Apply low-pass filtering to reduce high-frequency noise
% Butterworth filter parameters
fs_est = 1/median(diff(D(:,1)));  % Estimated sampling frequency
fc_acc = min(20, fs_est/4);       % Cutoff frequency for accelerometer
fc_gyro = min(30, fs_est/4);      % Cutoff frequency for gyroscope

if fs_est > 10  % Only apply filtering if sampling rate is reasonable
    try
        % Design filters
        [b_acc, a_acc] = butter(2, fc_acc/(fs_est/2), 'low');
        [b_gyro, a_gyro] = butter(2, fc_gyro/(fs_est/2), 'low');
        
        % Apply filtering
        ax = filtfilt(b_acc, a_acc, ax_temp);
        ay = filtfilt(b_acc, a_acc, ay_temp);
        az = filtfilt(b_acc, a_acc, az_temp);
        
        gx = filtfilt(b_gyro, a_gyro, gx_temp);
        gy = filtfilt(b_gyro, a_gyro, gy_temp);
        gz = filtfilt(b_gyro, a_gyro, gz_temp);
        
        fprintf('应用了低通滤波: acc_fc=%.1f Hz, gyro_fc=%.1f Hz\n', fc_acc, fc_gyro);
    catch
        % Fallback: use moving average if butter filter fails
        window = max(3, round(fs_est/20));  % ~50ms window
        ax = movmean(ax_temp, window);
        ay = movmean(ay_temp, window);
        az = movmean(az_temp, window);
        gx = movmean(gx_temp, window);
        gy = movmean(gy_temp, window);
        gz = movmean(gz_temp, window);
        fprintf('应用了移动平均滤波: window=%d samples\n', window);
    end
else
    % No filtering for low sampling rates
    ax = ax_temp; ay = ay_temp; az = az_temp;
    gx = gx_temp; gy = gy_temp; gz = gz_temp;
    fprintf('采样率较低，未应用滤波\n');
end

% Remove any remaining outliers
ax = remove_outliers(ax);
ay = remove_outliers(ay);
az = remove_outliers(az);
gx = remove_outliers(gx);
gy = remove_outliers(gy);
gz = remove_outliers(gz);

end

function data_clean = remove_outliers(data)
% Remove outliers using median absolute deviation
median_val = median(data);
mad_val = mad(data, 1);  % Median absolute deviation
threshold = 5 * mad_val;  % 5-sigma threshold

outliers = abs(data - median_val) > threshold;
data_clean = data;
if any(outliers)
    % Replace outliers with median
    data_clean(outliers) = median_val;
    fprintf('移除了 %d 个异常值\n', sum(outliers));
end
end