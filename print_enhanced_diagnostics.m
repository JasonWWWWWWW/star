function print_enhanced_diagnostics(diagnostics, t, state_hist)
% print_enhanced_diagnostics - Comprehensive diagnostic reporting

fprintf('\n=================== 增强诊断报告 ===================\n');

% Basic statistics
N = length(t);
duration = t(end) - t(1);
fs_est = (N-1) / duration;

fprintf('数据统计:\n');
fprintf('  样本数: %d\n', N);
fprintf('  持续时间: %.2f s\n', duration);
fprintf('  估计采样率: %.1f Hz\n', fs_est);

% Position and velocity analysis
pos = state_hist(:,1:3);
vel = state_hist(:,4:6);
acc_bias = state_hist(:,7:9);

final_pos = pos(end,:);
final_vel = vel(end,:);
final_bias = acc_bias(end,:);

max_pos = max(abs(pos));
max_vel = max(abs(vel));
pos_range = [min(pos); max(pos)];
vel_range = [min(vel); max(vel)];

fprintf('\n轨迹分析:\n');
fprintf('  最终位置: [%.2f %.2f %.2f] m\n', final_pos);
fprintf('  最终速度: [%.3f %.3f %.3f] m/s\n', final_vel);
fprintf('  最大位置: [%.2f %.2f %.2f] m\n', max_pos);
fprintf('  最大速度: [%.3f %.3f %.3f] m/s\n', max_vel);
fprintf('  位置范围: X[%.1f,%.1f] Y[%.1f,%.1f] Z[%.1f,%.1f] m\n', ...
    pos_range(1,1), pos_range(2,1), pos_range(1,2), pos_range(2,2), pos_range(1,3), pos_range(2,3));

% Bias analysis
fprintf('\n偏置分析:\n');
fprintf('  最终偏置: [%.4f %.4f %.4f] m/s²\n', final_bias);
fprintf('  偏置范围: [%.4f %.4f %.4f] 到 [%.4f %.4f %.4f] m/s²\n', ...
    min(acc_bias), max(acc_bias));

bias_stability = std(acc_bias(round(end/2):end,:));  % Stability in second half
fprintf('  偏置稳定性(后半段标准差): [%.4f %.4f %.4f] m/s²\n', bias_stability);

% ZUPT performance analysis
fprintf('\nZUPT性能分析:\n');
fprintf('  ZUPT缩放次数: %d\n', diagnostics.zupt_scaled_count);
fprintf('  ZUPT跳过次数: %d\n', diagnostics.skipped_zupt);
fprintf('  快速偏置修正次数: %d\n', diagnostics.quick_bias_applied);
fprintf('  偏置测量应用次数: %d\n', diagnostics.bias_applied_count);

if isfield(diagnostics, 'drift_corrections')
    fprintf('  漂移补偿次数: %d\n', diagnostics.drift_corrections);
end
if isfield(diagnostics, 'velocity_resets')
    fprintf('  速度重置次数: %d\n', diagnostics.velocity_resets);
end

% Innovation analysis
if ~isempty(diagnostics.innov_list)
    innov_stats = [min(diagnostics.innov_list), mean(diagnostics.innov_list), ...
                   median(diagnostics.innov_list), max(diagnostics.innov_list), std(diagnostics.innov_list)];
    fprintf('\n创新统计 (最小/均值/中位/最大/标准差):\n');
    fprintf('  [%.3f %.3f %.3f %.3f %.3f]\n', innov_stats);
    
    % Innovation trend analysis
    if length(diagnostics.innov_list) > 5
        early_innov = mean(diagnostics.innov_list(1:min(5, end)));
        late_innov = mean(diagnostics.innov_list(max(1,end-4):end));
        fprintf('  创新趋势: 早期=%.3f, 晚期=%.3f (差值=%.3f)\n', early_innov, late_innov, late_innov-early_innov);
    end
end

% Navigation acceleration residuals
if ~isempty(diagnostics.mean_acc_still_list)
    fprintf('\n导航系加速度残差分析:\n');
    mean_residual = mean(diagnostics.mean_acc_still_list, 1);
    std_residual = std(diagnostics.mean_acc_still_list, 0, 1);
    max_residual = max(abs(diagnostics.mean_acc_still_list));
    
    fprintf('  平均残差: [%.3f %.3f %.3f] m/s²\n', mean_residual);
    fprintf('  残差标准差: [%.3f %.3f %.3f] m/s²\n', std_residual);
    fprintf('  最大残差: [%.3f %.3f %.3f] m/s²\n', max_residual);
    
    residual_norm = sqrt(sum(diagnostics.mean_acc_still_list.^2, 2));
    fprintf('  残差范数: 均值=%.3f, 最大=%.3f m/s²\n', mean(residual_norm), max(residual_norm));
end

% Performance assessment
fprintf('\n性能评估:\n');
pos_error = norm(final_pos);
vel_error = norm(final_vel);
max_speed = max(sqrt(sum(vel.^2, 2)));

fprintf('  最终位置误差: %.2f m\n', pos_error);
fprintf('  最终速度误差: %.3f m/s\n', vel_error);
fprintf('  最大运动速度: %.2f m/s\n', max_speed);

% Performance rating
if pos_error < 10 && vel_error < 1
    rating = '优秀 ✓';
    color_code = '绿色';
elseif pos_error < 30 && vel_error < 3
    rating = '良好 ⚠';
    color_code = '黄色';
elseif pos_error < 100 && vel_error < 10
    rating = '一般 ⚠';
    color_code = '橙色';
else
    rating = '较差 ✗';
    color_code = '红色';
end

fprintf('  总体评级: %s (%s)\n', rating, color_code);

% Recommendations
fprintf('\n改进建议:\n');
if diagnostics.zupt_scaled_count > N*0.1  % More than 10% of ZUPT scaled
    fprintf('  - 创新缩放过多，考虑调整ZUPT阈值或观测噪声\n');
end

if vel_error > 2
    fprintf('  - 速度漂移较大，考虑增强漂移补偿或调整过程噪声\n');
end

if pos_error > 50
    fprintf('  - 位置漂移严重，检查偏置估计和ZUPT检测质量\n');
end

if ~isempty(diagnostics.mean_acc_still_list) && mean(sqrt(sum(diagnostics.mean_acc_still_list.^2, 2))) > 1
    fprintf('  - 静止时加速度残差较大，检查标定质量和偏置估计\n');
end

if diagnostics.bias_applied_count < diagnostics.zupt_scaled_count * 0.3
    fprintf('  - 偏置测量应用较少，考虑放宽偏置测量条件\n');
end

% Data quality assessment
fprintf('\n数据质量评估:\n');
if fs_est < 50
    fprintf('  - 采样率较低 (%.1f Hz)，建议 >100Hz\n', fs_est);
elseif fs_est > 200
    fprintf('  - 采样率较高 (%.1f Hz)，可考虑降采样\n', fs_est);
else
    fprintf('  - 采样率合适 (%.1f Hz) ✓\n', fs_est);
end

still_ratio = 0;
if isfield(diagnostics, 'still_times') && ~isempty(diagnostics.still_times)
    still_ratio = length(diagnostics.still_times) / N * 100;
end
fprintf('  - 静止时间比例: %.1f%%\n', still_ratio);

if still_ratio < 10
    fprintf('    建议: 增加更多静止段以改善偏置估计\n');
elseif still_ratio > 70
    fprintf('    建议: 数据中运动过少，考虑增加动态段\n');
end

fprintf('=================== 诊断报告结束 ===================\n\n');

end