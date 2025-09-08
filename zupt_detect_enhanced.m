function is_still = zupt_detect_enhanced(acc, gyro, dt, opts)
% zupt_detect_enhanced - Enhanced Zero Velocity Update detection
% Uses multiple criteria and adaptive thresholds for robust ZUPT detection

if nargin < 4, opts = struct(); end

% Enhanced default parameters
if ~isfield(opts,'window'), opts.window = 0.3; end                    % Longer window for stability
if ~isfield(opts,'min_still_time'), opts.min_still_time = 0.8; end    % Longer minimum duration
if ~isfield(opts,'acc_factor'), opts.acc_factor = 3; end              % More conservative
if ~isfield(opts,'gyro_factor'), opts.gyro_factor = 3; end            % More conservative
if ~isfield(opts,'use_percentile'), opts.use_percentile = true; end
if ~isfield(opts,'percentile_level'), opts.percentile_level = 15; end % Higher percentile
if ~isfield(opts,'gravity_check'), opts.gravity_check = true; end     % Enable gravity check
if ~isfield(opts,'consistency_check'), opts.consistency_check = true; end % Enable consistency check

N = size(acc,1);
win = max(1, round(opts.window / dt));
min_samples = max(1, round(opts.min_still_time / dt));

% ========== Feature Extraction ==========

% 1. Acceleration magnitude deviation from gravity
acc_mag = sqrt(sum(acc.^2,2));
g_nominal = 9.81;
acc_gravity_dev = abs(acc_mag - g_nominal);
acc_gravity_dev_smooth = movmean(acc_gravity_dev, win);

% 2. Acceleration variance (energy in deviations)
acc_var = zeros(N,1);
for i = 1:3
    acc_centered = acc(:,i) - movmean(acc(:,i), win);
    acc_var = acc_var + movmean(acc_centered.^2, win);
end

% 3. Gyroscope magnitude and variance
gyro_mag = sqrt(sum(gyro.^2,2));
gyro_var = movmean(gyro_mag.^2, win);

% 4. Combined acceleration jerk (rate of change)
acc_jerk = [0; sqrt(sum(diff(acc).^2, 2))];
acc_jerk_smooth = movmean(acc_jerk, win);

% ========== Adaptive Threshold Computation ==========

if opts.use_percentile
    % Use percentiles for adaptive thresholds
    p = opts.percentile_level;
    
    acc_var_thresh = prctile(acc_var, p) * opts.acc_factor;
    gyro_var_thresh = prctile(gyro_var, p) * opts.gyro_factor;
    acc_gravity_thresh = prctile(acc_gravity_dev_smooth, p * 2) + 0.5;  % Gravity deviation threshold
    acc_jerk_thresh = prctile(acc_jerk_smooth, p * 1.5) * 2;            % Jerk threshold
else
    % Use median-based thresholds
    acc_var_thresh = median(acc_var) * opts.acc_factor;
    gyro_var_thresh = median(gyro_var) * opts.gyro_factor;
    acc_gravity_thresh = median(acc_gravity_dev_smooth) + 0.5;
    acc_jerk_thresh = median(acc_jerk_smooth) * 2;
end

% Set minimum thresholds to avoid over-sensitivity
acc_var_thresh = max(acc_var_thresh, 0.02^2);           % Minimum 0.02 m/s² variance
gyro_var_thresh = max(gyro_var_thresh, (2*pi/180)^2);   % Minimum 2 deg/s variance
acc_gravity_thresh = max(acc_gravity_thresh, 0.3);       % Minimum 0.3 m/s² gravity deviation
acc_jerk_thresh = max(acc_jerk_thresh, 0.5);            % Minimum 0.5 m/s³ jerk

fprintf('ZUPT阈值: acc_var=%.4f, gyro_var=%.4f, gravity_dev=%.2f, jerk=%.2f\n', ...
    acc_var_thresh, gyro_var_thresh, acc_gravity_thresh, acc_jerk_thresh);

% ========== Multi-Criteria Detection ==========

% Primary criteria
is_still_acc = acc_var < acc_var_thresh;
is_still_gyro = gyro_var < gyro_var_thresh;

% Secondary criteria (optional)
is_still_gravity = true(N,1);
is_still_jerk = true(N,1);

if opts.gravity_check
    is_still_gravity = acc_gravity_dev_smooth < acc_gravity_thresh;
end

if opts.consistency_check
    is_still_jerk = acc_jerk_smooth < acc_jerk_thresh;
end

% Combine all criteria (AND logic)
is_still_raw = is_still_acc & is_still_gyro & is_still_gravity & is_still_jerk;

% ========== Temporal Consistency Filtering ==========

% Remove short spurious detections and gaps
is_still = false(N,1);

if any(is_still_raw)
    % Find continuous segments
    d = diff([0; is_still_raw; 0]);
    starts = find(d == 1);
    ends = find(d == -1) - 1;
    
    % Keep only segments longer than minimum duration
    valid_segments = [];
    for i = 1:length(starts)
        duration = ends(i) - starts(i) + 1;
        if duration >= min_samples
            valid_segments = [valid_segments; starts(i), ends(i)];
        end
    end
    
    % Apply valid segments
    for i = 1:size(valid_segments, 1)
        is_still(valid_segments(i,1):valid_segments(i,2)) = true;
    end
    
    % Fill small gaps between close segments (optional)
    if size(valid_segments, 1) > 1
        max_gap = round(0.3 / dt);  % Maximum 0.3s gap to fill
        for i = 1:size(valid_segments, 1)-1
            gap_start = valid_segments(i,2) + 1;
            gap_end = valid_segments(i+1,1) - 1;
            gap_length = gap_end - gap_start + 1;
            
            if gap_length > 0 && gap_length <= max_gap
                is_still(gap_start:gap_end) = true;
                fprintf('填补了长度为%.2fs的间隙\n', gap_length * dt);
            end
        end
    end
end

% ========== Quality Assessment ==========

still_ratio = sum(is_still) / N;
n_segments = size(valid_segments, 1);

fprintf('ZUPT检测完成: %.1f%% 静止时间, %d 个静止段\n', still_ratio*100, n_segments);

% Warn if detection seems unreasonable
if still_ratio < 0.1
    fprintf('警告: 静止时间过少 (%.1f%%), 可能需要调整阈值\n', still_ratio*100);
elseif still_ratio > 0.8
    fprintf('警告: 静止时间过多 (%.1f%%), 可能阈值过于宽松\n', still_ratio*100);
end

if n_segments > 20
    fprintf('警告: 静止段过多 (%d), 可能存在误检测\n', n_segments);
elseif n_segments == 0
    fprintf('警告: 未检测到静止段, 请检查数据或调整参数\n');
end

end