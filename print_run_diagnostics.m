function print_run_diagnostics(t, acc_body, acc_nav, is_still, state_hist, diagnostics)
% Compact run diagnostics similar to user's logs
if nargin < 6 || isempty(diagnostics)
    diagnostics = struct();
end

N = numel(t);
duration = t(end) - t(1);
fs_est = (N-1) / duration;

vel = state_hist(:,4:6);
pos = state_hist(:,1:3);

fprintf('=== run diagnostics (compact) ===\n');
fprintf(' samples=%d, duration=%.2f s, est fs=%.2f Hz\n', N, duration, fs_est);
fprintf(' acc_body mean(all) = [%.3f %.3f %.3f] m/s^2\n', mean(acc_body,1));
fprintf(' acc_nav mean(all)  = [%.3f %.3f %.3f] m/s^2\n', mean(acc_nav,1));
fprintf(' still samples = %d (%.2f%%)\n', sum(is_still), 100*sum(is_still)/N);

% if diagnostics contains still stats
try
    if isfield(diagnostics,'mean_acc_still_list') && ~isempty(diagnostics.mean_acc_still_list)
        mab = diagnostics.mean_acc_still_list;
        C = size(mab,1);
        fprintf('  ZUPT mean_a_nav first/mean/last = [%.3f %.3f %.3f] / [%.3f %.3f %.3f] / [%.3f %.3f %.3f]\n', ...
            mab(1,:), mean(mab,1), mab(end,:));
    end
catch
end

fprintf(' final vel (last) = [%.3f %.3f %.3f] m/s, max|vel| = [%.3f %.3f %.3f]\n', ...
    vel(end,1), vel(end,2), vel(end,3), max(abs(vel(:,1))), max(abs(vel(:,2))), max(abs(vel(:,3))));
fprintf(' final pos (last) = [%.3f %.3f %.3f] m, pos x range [%.3f, %.3f]\n', ...
    pos(end,1), pos(end,2), pos(end,3), min(pos(:,1)), max(pos(:,1)));

flds = {'zupt_scaled_count','skipped_zupt','quick_bias_applied'};
for i=1:numel(flds)
    f = flds{i};
    if isfield(diagnostics,f)
        fprintf(' diagnostics.%s = %d\n', f, diagnostics.(f));
    end
end
fprintf('=== end diagnostics ===\n');
end

