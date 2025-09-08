function init_bias_body = estimate_initial_bias_enhanced(ax_c, ay_c, az_c, q, is_still, params)
% estimate_initial_bias_enhanced - Enhanced initial bias estimation
% Uses multiple static periods and robust statistics

g = params.g;
N = length(ax_c);

% Find all static periods
still_indices = find(is_still);
if isempty(still_indices)
    fprintf('警告: 未发现静止段，使用前段数据估计初始偏置\n');
    % Use first portion of data
    useN = min(N, 100);
    init_bias_body = estimate_bias_from_period(ax_c(1:useN), ay_c(1:useN), az_c(1:useN), q(1:useN,:), g);
    return;
end

% Group consecutive still indices into segments
segments = group_consecutive_indices(still_indices);

% Filter segments by minimum length (at least 0.5 seconds worth of data)
dt = 0.01;  % Assume ~100Hz, adjust if needed
min_length = max(10, round(0.5 / dt));
valid_segments = segments(cellfun(@length, segments) >= min_length);

if isempty(valid_segments)
    fprintf('警告: 静止段过短，使用最长的静止段\n');
    [~, max_idx] = max(cellfun(@length, segments));
    valid_segments = segments(max_idx);
end

fprintf('找到 %d 个有效静止段用于偏置估计\n', length(valid_segments));

% Estimate bias for each segment
bias_estimates = zeros(length(valid_segments), 3);
quality_scores = zeros(length(valid_segments), 1);

for i = 1:length(valid_segments)
    segment_idx = valid_segments{i};
    
    % Extract data for this segment
    ax_seg = ax_c(segment_idx);
    ay_seg = ay_c(segment_idx);
    az_seg = az_c(segment_idx);
    q_seg = q(segment_idx, :);
    
    % Estimate bias for this segment
    [bias_seg, quality] = estimate_bias_from_period(ax_seg, ay_seg, az_seg, q_seg, g);
    
    bias_estimates(i, :) = bias_seg;
    quality_scores(i) = quality;
end

% Select best estimate based on quality scores
if length(valid_segments) == 1
    init_bias_body = bias_estimates(1, :);
    fprintf('使用单个静止段的偏置估计\n');
else
    % Use weighted average based on quality scores
    weights = quality_scores / sum(quality_scores);
    init_bias_body = sum(bias_estimates .* weights, 1);
    
    % Alternative: use median for robustness
    if any(quality_scores < 0.5)  % If some segments have poor quality
        init_bias_body = median(bias_estimates, 1);
        fprintf('使用中位数偏置估计 (存在低质量段)\n');
    else
        fprintf('使用加权平均偏置估计\n');
    end
end

% Sanity check: bias should be reasonable
if any(abs(init_bias_body) > 5)
    fprintf('警告: 估计的偏置过大 [%.3f %.3f %.3f], 进行限制\n', init_bias_body);
    init_bias_body = max(-3, min(3, init_bias_body));  % Clamp to ±3 m/s²
end

fprintf('最终初始偏置估计: [%.4f %.4f %.4f] m/s²\n', init_bias_body);

end

function [bias_estimate, quality] = estimate_bias_from_period(ax, ay, az, q, g)
% Estimate bias from a single static period

N = length(ax);
acc_body = [ax, ay, az];

% Compute mean body acceleration
mean_acc_body = mean(acc_body, 1)';

% Compute mean attitude (average rotation matrix)
C_sum = zeros(3, 3);
for i = 1:N
    C_sum = C_sum + quat2dcm(q(i, :));
end
C_mean = C_sum / N;

% Predicted gravity in body frame
g_body_predicted = C_mean' * [0; 0; g];

% Bias estimate: measured - predicted
bias_estimate = mean_acc_body - g_body_predicted;
bias_estimate = bias_estimate';  % Return as row vector

% Quality assessment
acc_std = std(acc_body, 0, 1);
max_std = max(acc_std);

% Angle between measured and predicted gravity
cos_angle = dot(mean_acc_body, g_body_predicted) / (norm(mean_acc_body) * norm(g_body_predicted));
cos_angle = max(-1, min(1, cos_angle));  % Clamp to valid range
angle_deg = acosd(cos_angle);

% Quality score (higher is better)
% Penalize high standard deviation and large angle
quality = exp(-max_std) * exp(-angle_deg/30);  % Exponential decay

end

function segments = group_consecutive_indices(indices)
% Group consecutive indices into cell array of segments

if isempty(indices)
    segments = {};
    return;
end

segments = {};
current_segment = indices(1);

for i = 2:length(indices)
    if indices(i) == indices(i-1) + 1
        % Consecutive index, add to current segment
        current_segment = [current_segment, indices(i)];
    else
        % Gap found, save current segment and start new one
        segments{end+1} = current_segment;
        current_segment = indices(i);
    end
end

% Don't forget the last segment
segments{end+1} = current_segment;

end

function C = quat2dcm(q)
% Convert quaternion to direction cosine matrix
% q = [w x y z]
w = q(1); x = q(2); y = q(3); z = q(4);
C = [1-2*(y^2+z^2),   2*(x*y - z*w),   2*(x*z + y*w);
     2*(x*y + z*w),   1-2*(x^2+z^2),   2*(y*z - x*w);
     2*(x*z - y*w),   2*(y*z + x*w),   1-2*(x^2+y^2)];
end