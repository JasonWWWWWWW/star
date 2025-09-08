function plot_results_enhanced(t, q, state_hist, is_still, diagnostics)
% plot_results_enhanced - Enhanced plotting with comprehensive analysis

N = length(t);
pos = state_hist(:,1:3);
vel = state_hist(:,4:6);
acc_bias = state_hist(:,7:9);

% Create comprehensive figure layout
figure('Name','增强INS结果分析','Position',[100 100 1400 1000]);

% ========== Attitude Plot ==========
subplot(3,4,1);
plot(t, q(:,1), 'b-', 'LineWidth', 1.2); hold on;
plot(t, q(:,2), 'r-', 'LineWidth', 1.2);
plot(t, q(:,3), 'g-', 'LineWidth', 1.2);
plot(t, q(:,4), 'm-', 'LineWidth', 1.2);
ylabel('四元数'); xlabel('时间 (s)'); 
legend('qw','qx','qy','qz','Location','best'); 
grid on; title('姿态 (四元数)');

% ========== Position Plot ==========
subplot(3,4,2);
plot(t, pos(:,1), 'b-', 'LineWidth', 1.5); hold on;
plot(t, pos(:,2), 'r-', 'LineWidth', 1.5);
plot(t, pos(:,3), 'g-', 'LineWidth', 1.5);
% Highlight ZUPT periods
still_idx = find(is_still);
if ~isempty(still_idx)
    plot(t(still_idx), pos(still_idx,1), 'bo', 'MarkerSize', 2);
    plot(t(still_idx), pos(still_idx,2), 'ro', 'MarkerSize', 2);
    plot(t(still_idx), pos(still_idx,3), 'go', 'MarkerSize', 2);
end
ylabel('位置 (m)'); xlabel('时间 (s)');
legend('X','Y','Z','Location','best');
grid on; title('位置轨迹');

% ========== Velocity Plot ==========
subplot(3,4,3);
plot(t, vel(:,1), 'b-', 'LineWidth', 1.5); hold on;
plot(t, vel(:,2), 'r-', 'LineWidth', 1.5);
plot(t, vel(:,3), 'g-', 'LineWidth', 1.5);
if ~isempty(still_idx)
    plot(t(still_idx), vel(still_idx,1), 'bo', 'MarkerSize', 2);
    plot(t(still_idx), vel(still_idx,2), 'ro', 'MarkerSize', 2);
    plot(t(still_idx), vel(still_idx,3), 'go', 'MarkerSize', 2);
end
ylabel('速度 (m/s)'); xlabel('时间 (s)');
legend('Vx','Vy','Vz','Location','best');
grid on; title('速度');

% ========== ZUPT Detection ==========
subplot(3,4,4);
plot(t, double(is_still), 'k-', 'LineWidth', 2);
ylim([-0.1 1.1]); ylabel('静止检测'); xlabel('时间 (s)');
title(sprintf('ZUPT检测 (%.1f%% 静止)', sum(is_still)/N*100));
grid on;

% ========== Accelerometer Bias Evolution ==========
subplot(3,4,5);
plot(t, acc_bias(:,1), 'b-', 'LineWidth', 1.5); hold on;
plot(t, acc_bias(:,2), 'r-', 'LineWidth', 1.5);
plot(t, acc_bias(:,3), 'g-', 'LineWidth', 1.5);
ylabel('加速度偏置 (m/s²)'); xlabel('时间 (s)');
legend('bx','by','bz','Location','best');
grid on; title('加速度偏置估计');

% ========== 3D Trajectory ==========
subplot(3,4,6);
plot3(pos(:,1), pos(:,2), pos(:,3), 'b-', 'LineWidth', 2); hold on;
plot3(pos(1,1), pos(1,2), pos(1,3), 'go', 'MarkerSize', 10, 'LineWidth', 2); % Start
plot3(pos(end,1), pos(end,2), pos(end,3), 'ro', 'MarkerSize', 10, 'LineWidth', 2); % End
if ~isempty(still_idx)
    plot3(pos(still_idx,1), pos(still_idx,2), pos(still_idx,3), 'ko', 'MarkerSize', 3);
end
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title('3D轨迹'); grid on; axis equal;
legend('轨迹','起点','终点','静止点','Location','best');

% ========== Innovation Analysis ==========
subplot(3,4,7);
if ~isempty(diagnostics.innov_list)
    plot(diagnostics.still_times, diagnostics.innov_list, 'ro-', 'LineWidth', 1.5);
    ylabel('创新范数'); xlabel('时间 (s)');
    title(sprintf('ZUPT创新 (平均=%.2f)', mean(diagnostics.innov_list)));
    grid on;
    
    % Add threshold line if available
    % if isfield(diagnostics, 'innovation_threshold')
    %     yline(diagnostics.innovation_threshold, 'r--', 'Threshold');
    % end
else
    text(0.5, 0.5, '无ZUPT数据', 'HorizontalAlignment', 'center');
    title('ZUPT创新分析');
end

% ========== Velocity Magnitude ==========
subplot(3,4,8);
vel_mag = sqrt(sum(vel.^2, 2));
plot(t, vel_mag, 'k-', 'LineWidth', 2); hold on;
if ~isempty(still_idx)
    plot(t(still_idx), vel_mag(still_idx), 'ro', 'MarkerSize', 3);
end
ylabel('速度大小 (m/s)'); xlabel('时间 (s)');
title(sprintf('速度大小 (最大=%.2f)', max(vel_mag)));
grid on;

% ========== Position Drift Analysis ==========
subplot(3,4,9);
pos_mag = sqrt(sum(pos.^2, 2));
plot(t, pos_mag, 'b-', 'LineWidth', 2); hold on;
if ~isempty(still_idx)
    plot(t(still_idx), pos_mag(still_idx), 'ro', 'MarkerSize', 2);
end
ylabel('位置大小 (m)'); xlabel('时间 (s)');
title(sprintf('位置漂移 (最终=%.1fm)', pos_mag(end)));
grid on;

% ========== Navigation Frame Acceleration Residuals ==========
subplot(3,4,10);
if ~isempty(diagnostics.mean_acc_still_list)
    plot(diagnostics.still_times, diagnostics.mean_acc_still_list(:,1), 'b.-', 'LineWidth', 1.5); hold on;
    plot(diagnostics.still_times, diagnostics.mean_acc_still_list(:,2), 'r.-', 'LineWidth', 1.5);
    plot(diagnostics.still_times, diagnostics.mean_acc_still_list(:,3), 'g.-', 'LineWidth', 1.5);
    ylabel('导航系加速度 (m/s²)'); xlabel('时间 (s)');
    legend('ax','ay','az','Location','best');
    title('静止时导航系加速度残差');
    grid on;
    
    % Add zero reference lines
    yline(0, 'k--', 'Alpha', 0.5);
else
    text(0.5, 0.5, '无静止段数据', 'HorizontalAlignment', 'center');
    title('导航系加速度残差');
end

% ========== Bias Convergence ==========
subplot(3,4,11);
bias_mag = sqrt(sum(acc_bias.^2, 2));
plot(t, bias_mag, 'k-', 'LineWidth', 2);
ylabel('偏置大小 (m/s²)'); xlabel('时间 (s)');
title(sprintf('偏置收敛 (最终=%.3f)', bias_mag(end)));
grid on;

% ========== Performance Summary ==========
subplot(3,4,12);
axis off;

% Calculate performance metrics
final_pos_error = norm(pos(end,:));
final_vel_error = norm(vel(end,:));
max_vel = max(sqrt(sum(vel.^2, 2)));
mean_still_acc = [];
if ~isempty(diagnostics.mean_acc_still_list)
    mean_still_acc = mean(sqrt(sum(diagnostics.mean_acc_still_list.^2, 2)));
end

% Performance summary text
summary_text = {
    '性能总结:',
    '',
    sprintf('最终位置误差: %.1f m', final_pos_error),
    sprintf('最终速度误差: %.2f m/s', final_vel_error),
    sprintf('最大速度: %.2f m/s', max_vel),
    '',
    sprintf('ZUPT统计:'),
    sprintf('  缩放次数: %d', diagnostics.zupt_scaled_count),
    sprintf('  跳过次数: %d', diagnostics.skipped_zupt),
    sprintf('  偏置修正: %d', diagnostics.bias_applied_count),
    '',
    sprintf('平均静止加速度: %.3f m/s²', mean_still_acc),
    '',
    '性能评级:'
};

% Add performance rating
if final_pos_error < 20 && final_vel_error < 2
    summary_text{end+1} = '✓ 优秀';
    color = 'green';
elseif final_pos_error < 50 && final_vel_error < 5
    summary_text{end+1} = '⚠ 良好';
    color = 'orange';
else
    summary_text{end+1} = '✗ 需改进';
    color = 'red';
end

text(0.05, 0.95, summary_text, 'Units', 'normalized', 'VerticalAlignment', 'top', ...
     'FontSize', 9, 'FontWeight', 'bold');

sgtitle('增强INS性能分析', 'FontSize', 14, 'FontWeight', 'bold');

% Additional detailed plots
create_detailed_analysis_plots(t, state_hist, is_still, diagnostics);

end

function create_detailed_analysis_plots(t, state_hist, is_still, diagnostics)
% Create additional detailed analysis plots

% Figure 2: Detailed bias and innovation analysis
figure('Name','详细偏置与创新分析','Position',[150 150 1200 800]);

pos = state_hist(:,1:3);
vel = state_hist(:,4:6);
acc_bias = state_hist(:,7:9);

% Bias evolution in detail
subplot(2,3,1);
plot(t, acc_bias(:,1), 'b-', 'LineWidth', 2); hold on;
plot(t, acc_bias(:,2), 'r-', 'LineWidth', 2);
plot(t, acc_bias(:,3), 'g-', 'LineWidth', 2);
still_idx = find(is_still);
if ~isempty(still_idx)
    scatter(t(still_idx), acc_bias(still_idx,1), 20, 'b', 'filled', 'Alpha', 0.6);
    scatter(t(still_idx), acc_bias(still_idx,2), 20, 'r', 'filled', 'Alpha', 0.6);
    scatter(t(still_idx), acc_bias(still_idx,3), 20, 'g', 'filled', 'Alpha', 0.6);
end
xlabel('时间 (s)'); ylabel('偏置 (m/s²)');
legend('bx','by','bz','Location','best');
title('加速度偏置演化'); grid on;

% Innovation statistics
subplot(2,3,2);
if ~isempty(diagnostics.innov_list)
    histogram(diagnostics.innov_list, 20, 'Normalization', 'probability');
    xlabel('创新范数'); ylabel('概率');
    title(sprintf('创新分布 (均值=%.2f, 标准差=%.2f)', ...
          mean(diagnostics.innov_list), std(diagnostics.innov_list)));
    grid on;
end

% Velocity during ZUPT
subplot(2,3,3);
if ~isempty(diagnostics.mean_vel_still_list)
    plot(diagnostics.still_times, diagnostics.mean_vel_still_list(:,1), 'b.-', 'LineWidth', 1.5); hold on;
    plot(diagnostics.still_times, diagnostics.mean_vel_still_list(:,2), 'r.-', 'LineWidth', 1.5);
    plot(diagnostics.still_times, diagnostics.mean_vel_still_list(:,3), 'g.-', 'LineWidth', 1.5);
    xlabel('时间 (s)'); ylabel('速度 (m/s)');
    legend('vx','vy','vz','Location','best');
    title('ZUPT时速度'); grid on;
end

% Position error growth
subplot(2,3,4);
pos_error = sqrt(sum(pos.^2, 2));
plot(t, pos_error, 'k-', 'LineWidth', 2); hold on;
if ~isempty(still_idx)
    plot(t(still_idx), pos_error(still_idx), 'ro', 'MarkerSize', 4);
end
xlabel('时间 (s)'); ylabel('位置误差 (m)');
title('位置误差增长'); grid on;

% Velocity error growth
subplot(2,3,5);
vel_error = sqrt(sum(vel.^2, 2));
plot(t, vel_error, 'k-', 'LineWidth', 2); hold on;
if ~isempty(still_idx)
    plot(t(still_idx), vel_error(still_idx), 'ro', 'MarkerSize', 4);
end
xlabel('时间 (s)'); ylabel('速度误差 (m/s)');
title('速度误差增长'); grid on;

% ZUPT effectiveness
subplot(2,3,6);
if ~isempty(diagnostics.mean_acc_still_list)
    acc_residual_norm = sqrt(sum(diagnostics.mean_acc_still_list.^2, 2));
    plot(diagnostics.still_times, acc_residual_norm, 'ko-', 'LineWidth', 2, 'MarkerSize', 6);
    xlabel('时间 (s)'); ylabel('加速度残差范数 (m/s²)');
    title('ZUPT有效性 (越小越好)'); grid on;
    
    % Add trend line
    if length(acc_residual_norm) > 3
        p = polyfit(diagnostics.still_times, acc_residual_norm, 1);
        trend = polyval(p, diagnostics.still_times);
        plot(diagnostics.still_times, trend, 'r--', 'LineWidth', 2);
        legend('实际值', '趋势线', 'Location', 'best');
    end
end

sgtitle('详细偏置与创新分析', 'FontSize', 14, 'FontWeight', 'bold');

end