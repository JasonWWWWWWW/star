function print_run_diagnostics()
% 生成与示例类似的简要统计，依赖 base workspace 变量
try
    t = evalin('base','t');
    acc_body = evalin('base','acc_body');
    acc_nav = evalin('base','acc_nav');
    is_still = evalin('base','is_still');
    state_hist = evalin('base','state_hist');
catch
    return;
end

N = numel(t);
dur = t(end) - t(1);
fs_est = (N-1)/dur;
vel = state_hist(:,4:6);
pos = state_hist(:,1:3);

fprintf('=== run diagnostics (compact) ===\n');
fprintf(' samples=%d, duration=%.2f s, est fs=%.2f Hz\n', N, dur, fs_est);
fprintf(' acc_body mean(all) = [%.3f %.3f %.3f] m/s^2\n', mean(acc_body,1));
fprintf(' acc_nav mean(all)  = [%.3f %.3f %.3f] m/s^2\n', mean(acc_nav,1));
fprintf(' still samples = %d (%.2f%%)\n', nnz(is_still), 100*nnz(is_still)/N);

% still-window quick check
if nnz(is_still) > 0
    idx = find(is_still);
    s0 = idx(1); s1 = idx(min(end, max(1, round(length(idx)*0.6))));
    accb_mean = mean(acc_body(s0:s1,:),1);
    % 预测 g_body: 使用均值姿态近似
    try q = evalin('base','q'); params = evalin('base','params'); catch, q=[]; params=struct('g',9.80665); end
    if ~isempty(q)
        Csum = zeros(3,3);
        for i=s0:s1, Csum = Csum + quat2dcm_local(q(i,:)); end
        Cmean = Csum / (s1-s0+1);
        g_body_pred = Cmean' * [0;0;params.g];
        ang = acosd( max(-1,min(1, dot(accb_mean',g_body_pred)/(norm(accb_mean)*norm(g_body_pred)+eps))) );
    else
        g_body_pred = [NaN;NaN;NaN]; ang = NaN;
    end
    fprintf('  still mean acc_body = [%.3f %.3f %.3f], predicted g_body = [%.3f %.3f %.3f], angle=%.2f deg\n', ...
        accb_mean, g_body_pred, ang);
end

v_last = vel(end,:); vmax = max(abs(vel),[],1);
px = pos(:,1); fprintf(' final vel (last) = [%.3f %.3f %.3f] m/s, max|vel| = [%.3f %.3f %.3f]\n', v_last, vmax);
fprintf(' final pos (last) = [%.3f %.3f %.3f] m, pos x range [%.3f, %.3f]\n', pos(end,:), min(px), max(px));
fprintf('=== end diagnostics ===\n');

end

function C = quat2dcm_local(q)
w=q(1); x=q(2); y=q(3); z=q(4);
C = [1-2*(y^2+z^2),   2*(x*y - z*w),   2*(x*z + y*w);
     2*(x*y + z*w),   1-2*(x^2+z^2),   2*(y*z - x*w);
     2*(x*z - y*w),   2*(y*z + x*w),   1-2*(x^2+y^2)];
end

