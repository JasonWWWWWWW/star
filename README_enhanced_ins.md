# 增强型哑铃惯性导航系统

## 概述

这是一个增强型的哑铃惯性导航系统，专门针对手机IMU数据进行优化。相比原始版本，新版本在以下方面有显著改进：

### 主要改进

1. **增强数据预处理**
   - 自适应低通滤波
   - 异常值检测和去除
   - 坐标系映射优化

2. **改进的IMU标定**
   - 多静止段标定
   - 椭球拟合算法
   - 重力基准标定
   - 标定质量评估

3. **自适应姿态估计**
   - 基于运动状态的自适应Madgwick算法
   - 改进的滤波策略
   - 四元数连续性保证

4. **增强ZUPT检测**
   - 多准则检测（加速度方差、陀螺方差、重力偏差、加速度跳变）
   - 自适应阈值
   - 时间一致性滤波

5. **鲁棒卡尔曼滤波**
   - 自适应过程噪声
   - 多重偏置修正机制
   - 漂移补偿
   - 创新门限自适应缩放

6. **全面诊断分析**
   - 详细性能评估
   - 多维度可视化
   - 改进建议

## 文件结构

```
enhanced_ins/
├── main_demo_improved.m              # 主程序
├── preprocess_imu_data.m            # 数据预处理
├── calibrate_imu_enhanced.m         # 增强IMU标定
├── estimate_attitude_enhanced.m     # 增强姿态估计
├── zupt_detect_enhanced.m           # 增强ZUPT检测
├── estimate_initial_bias_enhanced.m # 增强初始偏置估计
├── ins_propagate_kf_enhanced.m      # 增强INS卡尔曼滤波
├── plot_results_enhanced.m          # 增强结果可视化
├── print_enhanced_diagnostics.m     # 增强诊断报告
└── README_enhanced_ins.md           # 本文档
```

## 使用方法

### 1. 数据准备

确保你的CSV数据文件格式为：
```
t, ax, ay, az, gx, gy, gz
```
其中：
- `t`: 时间戳 (秒)
- `ax, ay, az`: 加速度计读数 (m/s²)
- `gx, gy, gz`: 陀螺仪读数 (rad/s)

### 2. 运行主程序

```matlab
% 在MATLAB中运行
main_demo_improved
```

### 3. 参数调整

主要参数在 `main_demo_improved.m` 中：

#### KF参数
```matlab
params.g = 9.80665;                    % 重力加速度
params.Q_gyro_bias = (0.002*deg2rad)^2; % 陀螺偏置过程噪声
params.Q_acc_bias  = (0.02)^2;          % 加速度偏置过程噪声
params.Q_vel       = 5e-5;              % 速度过程噪声
params.R_zupt_acc  = 0.5;               % ZUPT观测噪声
```

#### ZUPT检测参数
```matlab
zupt_opts.window = 0.3;              % 检测窗口长度 (秒)
zupt_opts.min_still_time = 0.8;      % 最小静止时间 (秒)
zupt_opts.acc_factor = 3;            % 加速度阈值因子
zupt_opts.gyro_factor = 3;           % 陀螺阈值因子
```

#### KF选项
```matlab
opts_kf.VEL_INNOV_GATE = 2.5;        % 速度创新门限
opts_kf.MAX_DX_VEL = 1.5;            % 最大速度更新
opts_kf.MAX_DX_BIAS = 1.0;           % 最大偏置更新
opts_kf.bias_meas_R = 0.02;          # 偏置测量噪声
opts_kf.drift_compensation = true;    # 启用漂移补偿
```

## 性能指标

### 评级标准

- **优秀 ✓**: 最终位置误差 < 10m, 速度误差 < 1m/s
- **良好 ⚠**: 最终位置误差 < 30m, 速度误差 < 3m/s  
- **一般 ⚠**: 最终位置误差 < 100m, 速度误差 < 10m/s
- **较差 ✗**: 超出上述范围

### 典型性能

对于20秒的哑铃运动数据：
- 位置漂移: 通常 < 50m
- 速度漂移: 通常 < 5m/s
- 偏置稳定性: < 0.1 m/s²

## 故障排除

### 1. 高位置漂移

**可能原因：**
- 初始偏置估计不准确
- ZUPT检测质量差
- 标定不充分

**解决方案：**
- 增加数据开始时的静止时间
- 调整ZUPT检测参数
- 检查坐标系映射

### 2. 高速度漂移

**可能原因：**
- 过程噪声设置不当
- 创新门限过严
- 偏置修正不足

**解决方案：**
- 增大 `params.Q_vel`
- 放宽 `opts_kf.VEL_INNOV_GATE`
- 启用 `opts_kf.drift_compensation`

### 3. ZUPT检测过多/过少

**过多检测：**
- 减小 `zupt_opts.acc_factor` 和 `zupt_opts.gyro_factor`
- 减小 `zupt_opts.window`

**过少检测：**
- 增大阈值因子
- 增大检测窗口
- 减小 `zupt_opts.min_still_time`

### 4. 偏置不收敛

**解决方案：**
- 检查静止段质量
- 调整 `opts_kf.bias_meas_R`
- 放宽 `opts_kf.TRUST_BIAS_MEAS_ANGLE_DEG`

## 高级调优

### 1. 数据特定优化

根据你的具体数据特征调整参数：

```matlab
% 高噪声数据
params.R_zupt_acc = 1.0;  % 增大观测噪声
zupt_opts.acc_factor = 5; % 更保守的ZUPT

% 低噪声数据  
params.R_zupt_acc = 0.2;  % 减小观测噪声
zupt_opts.acc_factor = 2; % 更敏感的ZUPT
```

### 2. 运动模式优化

```matlab
% 缓慢运动
params.Q_vel = 1e-5;           % 减小速度过程噪声
opts_kf.velocity_damping = 0.95; % 更强的速度阻尼

% 快速运动
params.Q_vel = 1e-4;           % 增大速度过程噪声  
opts_kf.velocity_damping = 0.99; % 较弱的速度阻尼
```

## 输出解释

### 1. 图形输出

- **姿态图**: 四元数时间序列
- **位置图**: 3D轨迹和时间序列
- **速度图**: 速度分量和幅值
- **ZUPT检测**: 静止段标识
- **偏置演化**: 偏置估计收敛过程
- **创新分析**: KF性能指标
- **性能总结**: 综合评估

### 2. 诊断报告

包含详细的性能统计、问题诊断和改进建议。

## 扩展功能

### 1. 多传感器融合

可以扩展支持磁力计：

```matlab
% 在 estimate_attitude_enhanced.m 中添加磁力计支持
% 修改 Madgwick 算法包含磁力计观测
```

### 2. 在线标定

可以实现运行时的在线偏置估计：

```matlab
% 在 ins_propagate_kf_enhanced.m 中
% 增加连续偏置更新机制
```

### 3. 机器学习优化

可以使用ML方法优化参数：

```matlab
% 使用历史数据训练参数选择模型
% 实现自适应参数调整
```

## 参考文献

1. Madgwick, S. (2010). An efficient orientation filter for IMU and MARG sensor arrays.
2. Foxlin, E. (2005). Pedestrian tracking with shoe-mounted inertial sensors.
3. Skog, I. (2010). Zero-velocity detection—An algorithm evaluation.

## 版本历史

- v1.0: 基础INS实现
- v2.0: 增强版本，包含所有改进功能

## 联系信息

如有问题或建议，请通过以下方式联系：
- 技术支持: [your-email@example.com]
- 项目主页: [your-project-url]