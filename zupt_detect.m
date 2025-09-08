function is_still = zupt_detect(acc, gyro, dt, opts)

if nargin < 4, opts = struct(); end

if ~isfield(opts,'window'), opts.window = 0.2; end
if ~isfield(opts,'min_still_time'), opts.min_still_time = 0.5; end
if ~isfield(opts,'acc_factor'), opts.acc_factor = 5; end
if ~isfield(opts,'gyro_factor'), opts.gyro_factor = 5; end
if ~isfield(opts,'use_percentile'), opts.use_percentile = true; end

N = size(acc,1);
win = max(1, round(opts.window / dt));

% 加速度模长去中心化后的局部能量
acc_mag = sqrt(sum(acc.^2,2));
acc_mean_win = movmean(acc_mag, win);
acc_dev = (acc_mag - acc_mean_win).^2;
acc_rms = movmean(acc_dev, win);

% 角速度能量
gyro_mag = sqrt(sum(gyro.^2,2));
gyro_rms = movmean(gyro_mag.^2, win);

% 自适应阈值
if opts.use_percentile
    acc_base = prctile(acc_rms, 10);
    gyro_base = prctile(gyro_rms, 10);
else
    acc_base = median(acc_rms);
    gyro_base = median(gyro_rms);
end

acc_thr = max(acc_base * opts.acc_factor,  (0.01)^2 );
gyro_thr = max(gyro_base * opts.gyro_factor, ( (1*pi/180)^2 ) );

is_still_raw = (acc_rms < acc_thr) & (gyro_rms < gyro_thr);

% 要求至少一定长度的连续 true
min_samples = max(1, round(opts.min_still_time / dt));
is_still = false(N,1);
if any(is_still_raw)
    d = diff([0; is_still_raw; 0]);
    starts = find(d==1);
    ends = find(d==-1)-1;
    for i=1:length(starts)
        len = ends(i) - starts(i) + 1;
        if len >= min_samples
            is_still(starts(i):ends(i)) = true;
        end
    end
end
end

