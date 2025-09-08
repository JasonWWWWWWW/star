% example_usage.m - 增强型INS使用示例

% 清理工作空间
clear; close all; clc;

fprintf('=== 增强型哑铃惯性导航系统使用示例 ===\n\n');

%% 示例1: 基本使用 (默认配置)
fprintf('示例1: 使用默认配置\n');
fprintf('--------------------\n');

% 假设你的数据文件名为 'merged_sensor_data.csv'
dataFile = 'merged_sensor_data.csv';

if isfile(dataFile)
    % 使用默认配置运行
    run_enhanced_ins(dataFile, 'default');
else
    fprintf('数据文件 %s 不存在，跳过示例1\n\n', dataFile);
end

%% 示例2: 高噪声数据配置
fprintf('示例2: 高噪声数据配置\n');
fprintf('--------------------\n');

if isfile(dataFile)
    % 使用高噪声配置
    run_enhanced_ins(dataFile, 'high_noise');
else
    fprintf('数据文件 %s 不存在，跳过示例2\n\n', dataFile);
end

%% 示例3: 手动配置调整
fprintf('示例3: 手动配置调整\n');
fprintf('------------------\n');

if isfile(dataFile)
    % 获取默认配置
    config = config_enhanced_ins('default');
    
    % 手动调整参数
    config.params.Q_vel = 1e-4;              % 增大速度过程噪声
    config.opts_kf.VEL_INNOV_GATE = 3.0;     % 放宽创新门限
    config.zupt_opts.acc_factor = 2.5;       % 更敏感的ZUPT检测
    config.opts_kf.drift_compensation = true; % 启用漂移补偿
    
    fprintf('使用自定义配置运行...\n');
    
    % 你需要手动运行完整的处理流程
    % 这里只是展示如何获取和修改配置
    fprintf('配置已修改，请参考 main_demo_improved.m 了解完整流程\n\n');
else
    fprintf('数据文件 %s 不存在，跳过示例3\n\n', dataFile);
end

%% 示例4: 不同运动模式
fprintf('示例4: 不同运动模式配置\n');
fprintf('----------------------\n');

scenarios = {'slow_motion', 'fast_motion', 'short_test', 'long_test'};

for i = 1:length(scenarios)
    scenario = scenarios{i};
    fprintf('配置场景: %s\n', scenario);
    
    if isfile(dataFile)
        fprintf('  运行中...\n');
        % run_enhanced_ins(dataFile, scenario);
        fprintf('  完成 (已注释以避免重复运行)\n');
    else
        fprintf('  跳过 (数据文件不存在)\n');
    end
end

%% 示例5: 调试模式
fprintf('\n示例5: 调试模式\n');
fprintf('--------------\n');

if isfile(dataFile)
    fprintf('使用调试模式运行 (包含详细诊断信息):\n');
    % run_enhanced_ins(dataFile, 'debug');
    fprintf('调试模式已注释以避免重复运行\n');
else
    fprintf('数据文件 %s 不存在，跳过调试模式\n', dataFile);
end

%% 性能比较示例
fprintf('\n示例6: 性能比较\n');
fprintf('--------------\n');

if isfile(dataFile)
    fprintf('比较不同配置的性能:\n');
    
    scenarios_compare = {'default', 'high_noise', 'slow_motion'};
    
    for i = 1:length(scenarios_compare)
        scenario = scenarios_compare{i};
        fprintf('\n配置: %s\n', scenario);
        
        % 这里你可以运行并收集结果进行比较
        % [state_hist, diagnostics] = run_enhanced_ins_silent(dataFile, scenario);
        % performance_results{i} = analyze_performance(state_hist, diagnostics);
        
        fprintf('  (示例代码 - 需要实现 run_enhanced_ins_silent 函数)\n');
    end
else
    fprintf('数据文件不存在，无法进行性能比较\n');
end

%% 参数敏感性分析示例
fprintf('\n示例7: 参数敏感性分析\n');
fprintf('--------------------\n');

fprintf('分析关键参数对性能的影响:\n');

% 定义要测试的参数范围
Q_vel_values = [1e-5, 5e-5, 1e-4, 2e-4];
VEL_INNOV_GATE_values = [1.5, 2.0, 2.5, 3.0];

fprintf('Q_vel 测试值: ');
fprintf('%.1e ', Q_vel_values);
fprintf('\n');

fprintf('VEL_INNOV_GATE 测试值: ');
fprintf('%.1f ', VEL_INNOV_GATE_values);
fprintf('\n');

fprintf('(完整的参数敏感性分析需要实现专门的测试函数)\n');

%% 实用提示
fprintf('\n=== 实用提示 ===\n');
fprintf('1. 数据质量检查:\n');
fprintf('   - 确保数据文件格式正确: [t, ax, ay, az, gx, gy, gz]\n');
fprintf('   - 检查采样率 (建议 50-200 Hz)\n');
fprintf('   - 确保有足够的静止段用于ZUPT\n\n');

fprintf('2. 配置选择指南:\n');
fprintf('   - 默认配置: 大多数情况下的起点\n');
fprintf('   - high_noise: 数据噪声较大时\n');
fprintf('   - slow_motion: 缓慢运动时\n');
fprintf('   - fast_motion: 快速运动时\n');
fprintf('   - debug: 需要详细诊断信息时\n\n');

fprintf('3. 性能调优:\n');
fprintf('   - 查看诊断报告中的改进建议\n');
fprintf('   - 根据具体数据特征调整参数\n');
fprintf('   - 使用多个配置进行比较\n\n');

fprintf('4. 常见问题解决:\n');
fprintf('   - 位置漂移大: 检查ZUPT质量和初始标定\n');
fprintf('   - 速度漂移大: 启用漂移补偿\n');
fprintf('   - ZUPT检测不准: 调整检测阈值\n\n');

fprintf('=== 示例结束 ===\n');

%% 清理函数 (可选)
function cleanup_example()
    % 清理示例生成的文件和变量
    fprintf('清理示例文件...\n');
    % 删除临时文件等
end