# YOLO11 Buff检测与EKF跟踪系统技术文档

## 项目介绍

这是一个完整的**机器人视觉Buff跟踪系统**，集成了YOLO11目标检测和EKF（扩展卡尔曼滤波）运动预测算法。系统主要用于RoboMaster等机器人竞赛中自动识别和跟踪旋转的Buff目标。

### 系统架构
- **检测层**：YOLO11模型检测Buff目标和关键点
- **跟踪层**：EKF算法预测Buff运动轨迹
- **决策层**：根据预测结果控制机器人射击

## YOLO11检测模块分析

### 1. 文件意义和作用

`yolo11_buff.cpp`主要负责**Buff目标检测**，其核心功能：
- Buff目标检测和定位
- 6个关键点提取
- 边界框回归和置信度评估
- 非极大值抑制（NMS）处理多目标

### 2. 算法实现

#### 检测网络输出格式 `[15, 8400]`：
```
[0-3]:   边界框参数 [cx, cy, ow, oh]
[4]:     置信度分数
[5-14]:  6个关键点坐标 (x1,y1,x2,y2,...x6,y6)
```

#### 检测流程：
1. **图像预处理**：Letterbox变换保持宽高比
2. **模型推理**：OpenVINO框架CPU推理
3. **后处理**：置信度阈值筛选 + NMS
4. **坐标映射**：将检测结果映射回原图

## EKF运动预测模块分析

### 1. 扩展卡尔曼滤波器框架

**文件**：`tools/extended_kalman_filter.hpp`

核心功能：
- 非线性状态估计
- 多速率观测更新
- 卡方检验异常检测
- 支持自定义状态转移和观测函数

### 2. Buff运动建模

#### A. 小符模型（SmallTarget）
**状态向量（7维）**：`[R_yaw, v_R_yaw, R_pitch, R_dis, yaw, angle, spd]`

**运动特性**：
- 恒定角速度：`CV_PI/3 rad/s` (约1.047 rad/s)
- 角速度方向检测：使用Voting机制判断顺逆时针
- 状态方程：线性模型 + 过程噪声

**关键算法**：
```cpp
// 状态预测
A << 1.0;  // 状态转移矩阵
B << SMALL_W * (cw_ccw > 0 ? 1 : -1);  // 角速度输入

// 非线性预测函数
[&](const Eigen::VectorXd & x) {
    return A * x + deltatime * B;
}
```

#### B. 大符模型（BigTarget）
**状态向量（10维）**：`[R_yaw, v_R_yaw, R_pitch, R_dis, yaw, angle, spd, a, w, fi]`

**运动特性**：
- 正弦速度模型：`spd = a*sin(w*t + fi) + 2.09 - a`
- 参数范围：`a∈[0.78,1.045], w∈[1.884,2.000]`
- 非线性状态转移

**预测器实现**：
```cpp
// 非线性状态转移函数
[&](const Eigen::VectorXd & x) {
    double a = x[2], w = x[3], sita = x[4];
    Eigen::VectorXd m(5);
    m << x[0] + (cw_ccw > 0 ? 1 : -1) * (-a/w*cos(sita + w*deltatime) + a/w*cos(sita) + (2.09-a)*deltatime),
         a*sin(sita + w*deltatime) + 2.09 - a,
         a, w, sita + w*deltatime;
    return m;
}
```

### 3. Buff预测器设计

#### 预测器架构

**基类**：`Predictor` - 定义预测器接口
- `update(angle, nowtime)`: 更新观测
- `predict(delta_time)`: 预测未来状态
- `is_unsolve()`: 判断滤波状态

**派生类**：针对不同的Buff运动模型

#### A. Small_Predictor（小符预测器）
- **状态维度**：1维（角度）
- **模型类型**：恒定角速度线性模型
- **预测方程**：`angle += (±SMALL_W) * delta_time`

#### B. Big_Predictor（大符预测器）
- **状态维度**：5维 `[angle, spd, a, w, fi]`
- **模型类型**：正弦速度非线性模型
- **预测方程**：基于正弦函数的复杂非线性变换

#### C. XYZ_predictor（位置预测器）
- **状态向量**：3维 `[x, y, z]`
- **模型类型**：标准线性卡尔曼滤波
- **应用场景**：坐标平滑去噪

### 4. 关键算法特性

#### 状态估计特点：

1. **多观测批次更新**：
```cpp
// 分离测量更新（角度、位置分开处理）
update_angle_measurement(z_angle, H_angle, R_angle);
update_position_measurement(z_pos, H_pos, R_pos);
```

2. **非线性观测处理**：
```cpp
// 使用雅可比矩阵线性化观测函数
Eigen::MatrixXd h_jacobian() const {
    // 观测函数的偏导数矩阵
}
```

3. **角度周期约束**：
```cpp
// 处理角度周期性（-π, π]）
angle = limit_rad(angle);
```

4. **异常检测机制**：
```cpp
// 跳变检测和状态复位
if (abs(angle_jump) > threshold) {
    reset_filter_state();
}
```

#### 自适应参数拟合：

**大符速度拟合**：
```cpp
// 使用RANSAC算法拟合速度正弦函数
tools::RansacSineFitter spd_fitter_;
double fit_spd_ = spd_fitter_.fit(speed_history);
```

## 系统集成

### 1. 数据流
```
摄像头 → YOLO11检测 → Buff目标提取 → EKF滤波 → 轨迹预测 → 射击控制
```

### 2. 时序处理
```cpp
// 检测阶段：获取当前Buff状态
auto objects = yolo11_buff.get_multicandidateboxes(image);

// 跟踪阶段：滤波和预测
buff_target->get_target(buff_opt, timestamp);
buff_target->predict(shoot_delay);

// 决策阶段：计算射击角度
auto aim_point = calculate_predication_point(buff_target, shoot_delay);
```

### 3. 性能优化

- **实时性**：OpenVINO CPU推理，支持60FPS
- **鲁棒性**：异常检测和自动复位机制
- **精度**：亚像素级关键点检测 + 毫米级轨迹预测

## 技术亮点

1. **分层架构**：检测与跟踪分离，模块清晰
2. **多模型适配**：小符/大符分别建模，提高预测精度
3. **工程化EKF**：完全的C++实现，集成异常处理和统计检验
4. **实时性能**：优化算法复杂度，满足实时性要求

这套系统展现了机器人视觉领域的典型技术栈：深度学习检测 + 传统滤波算法跟踪 + 实时控制决策，是RoboMaster等竞赛中的核心技术方案。