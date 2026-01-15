# 大符快速连续击打功能使用说明

## 功能概述

实现了大符模式下的快速连续击打功能，通过主动切换目标，充分利用"击中第一个后1秒内可以击打第二个"的规则，将每组击打时间从3-4秒缩短到1-2秒。

## 核心特性

### 1. 状态机管理

```cpp
enum class BigRuneHitPhase {
  FIRST_TARGET,    // 准备击打第1个目标
  SECOND_TARGET,   // 准备击打第2个目标
  WAIT_RESET       // 等待重新随机
};
```

### 2. 自动目标切换

- **主动切换**：基于预测时间自动切换，无需等待击中反馈
- **时间估计**：考虑子弹飞行时间、系统延迟和安全裕量
- **智能选择**：根据旋转方向选择最优击打目标

### 3. 周期检测

自动检测新的2扇叶周期，重置击打策略。

## 使用方法

### 方法1：使用新的 `detect_fast_big_rune()` 方法

```cpp
#include "buff_detector.hpp"

// 创建检测器
auto_buff::Buff_Detector detector(config_path);

// 在主循环中使用
cv::Mat img = /* 获取图像 */;
double bullet_speed = 24.0;  // 子弹速度 m/s

// 使用快速击打模式
auto powerrune = detector.detect_fast_big_rune(img, bullet_speed);

if (powerrune.has_value()) {
    // 正常处理击打逻辑
    auto& target = powerrune->target();
    // ... 弹道解算、云台控制等
}
```

### 方法2：修改现有代码切换到快速模式

在现有的buff任务代码中，将：

```cpp
auto powerrune = detector.detect(img);
```

替换为：

```cpp
auto powerrune = detector.detect_fast_big_rune(img, bullet_speed);
```

## 参数调优

### 击打时间估计

在 `buff_detector.cpp:estimate_hit_time()` 中调整：

```cpp
double Buff_Detector::estimate_hit_time(double distance, double bullet_speed)
{
  double flight_time = distance / bullet_speed;

  // 根据实际系统延迟调整这些参数
  const double SYSTEM_DELAY = 0.15;  // 检测+处理延迟 (秒)
  const double SAFETY_MARGIN = 0.1;   // 安全裕量 (秒)

  return flight_time + SYSTEM_DELAY + SAFETY_MARGIN;
}
```

**调优建议：**
- **SYSTEM_DELAY**：实际测量从图像采集到云台开始响应的时间
- **SAFETY_MARGIN**：根据击打成功率调整（太激进可能导致切换过早）

### 切换时机

在 `buff_type.hpp:BigRuneFastHitStrategy::update()` 中：

```cpp
void update(double estimated_hit_time, double max_second_window = 1.0)
```

**参数说明：**
- `estimated_hit_time`：估计的击中时间，达到此时间后切换到第2个目标
- `max_second_window`：第2个目标的最大击打窗口（规则为1秒）

**调优建议：**
- 如果第1个目标击打率低，**增大** `estimated_hit_time`
- 如果第2个目标经常超时，**减小** `max_second_window`

## 工作流程

### 正常流程（2个点亮扇叶）

```
[检测到2个扇叶]
    ↓
[进入FIRST_TARGET阶段]
    ↓
[选择最优第1个目标] (离图像中心最近 / 根据旋转方向)
    ↓
[等待 estimated_hit_time] (约0.3-0.5秒)
    ↓
[自动切换到SECOND_TARGET阶段]
    ↓
[选择第2个目标]
    ↓
[在1秒窗口内击打]
    ↓
[等待重新随机] 或 [检测到新周期]
```

### 异常处理

1. **扇叶数量变化**
   - 从2变为其他值：进入小符模式或等待
   - 从其他值变为2：重置策略，开始新的击打周期

2. **目标丢失**
   - 使用原有的 `handle_lose()` 机制
   - 连续丢失20帧后进入 `LOSE` 状态

3. **超时处理**
   - 第1个目标超时：继续等待（不会自动切换）
   - 第2个目标超时（超过1秒）：进入 `WAIT_RESET` 状态

## 调试输出

### 日志级别设置

在代码中已添加调试日志：

```cpp
tools::logger()->debug("[Buff_Detector] 大符模式 - 阶段: {}, 目标索引: {}/{}",
  static_cast<int>(strategy.phase),
  target_idx,
  results.size() - 1);
```

### 监控指标

1. **阶段状态**：`strategy.phase`
   - 0: FIRST_TARGET
   - 1: SECOND_TARGET
   - 2: WAIT_RESET

2. **目标索引**：当前选择的目标索引（0或1）

3. **切换时间**：从第1个目标切换到第2个目标的时间

## 性能优化建议

### 1. 目标选择策略优化

当前默认使用"离图像中心最近"策略，可以根据实际情况修改：

```cpp
int Buff_Detector::select_target_by_rotation(
  const std::vector<YOLO11_BUFF::Object> & results,
  int rotation_direction)
{
  // 方案A：选择运动方向最有利的
  // 方案B：选择距离最近的
  // 方案C：选择预测位置最准的
}
```

### 2. 时间估计优化

如果条件允许，可以加入更精确的时间估计：

```cpp
double estimate_hit_time(double distance, double bullet_speed)
{
  // 考虑云台最大转速
  double gimbal_delay = calculate_gimbal_delay(target_angle);

  // 考虑当前云台状态
  double current_angle = get_current_gimbal_angle();
  double target_angle = get_target_angle();
  double rotation_needed = abs(target_angle - current_angle);

  return flight_time + gimbal_delay + SYSTEM_DELAY + SAFETY_MARGIN;
}
```

### 3. 自适应参数调整

根据实际击打效果动态调整参数：

```cpp
if (last_hit_successful) {
  // 成功击打，保持或略微提前切换时间
  switching_factor *= 0.98;
} else {
  // 未击中，延后切换时间
  switching_factor *= 1.02;
}
```

## 测试建议

### 1. 单元测试

- 测试状态机转换逻辑
- 测试周期检测功能
- 测试目标选择算法

### 2. 实车测试

**测试步骤：**
1. 固定距离测试（如5米）
2. 记录每组击打时间
3. 调整 `estimated_hit_time` 参数
4. 优化目标选择策略

**成功指标：**
- 每组击打时间 < 2秒
- 第1个目标命中率 > 80%
- 第2个目标命中率 > 70%
- 切换成功率 > 95%

### 3. 压力测试

- 不同距离测试（3m, 5m, 7m, 10m）
- 不同旋转速度测试
- 不同光照条件测试

## 故障排查

### 问题1：切换过早

**现象**：第1个目标未击中就切换到第2个

**解决**：
- 增大 `estimated_hit_time`
- 增大 `SAFETY_MARGIN`

### 问题2：切换过晚

**现象**：第2个目标经常超时

**解决**：
- 减小 `estimated_hit_time`
- 检查子弹速度是否准确
- 减小 `SYSTEM_DELAY`

### 问题3：目标选择错误

**现象**：总是选择较难击打的目标

**解决**：
- 修改 `select_target_by_rotation()` 策略
- 优化目标评分函数

### 问题4：周期检测失效

**现象**：不重置策略，连续击打同一个目标

**解决**：
- 检查 `detect_new_cycle()` 逻辑
- 确保 `last_light_num_` 正确更新

## 与原有功能的兼容性

- ✅ 不影响小符模式（`light_num == 1`）
- ✅ 不影响原有的 `detect()` 方法
- ✅ 可以通过参数控制是否启用快速模式
- ✅ 完全兼容现有的PnP、EKF、弹道解算模块

## 后续优化方向

1. **反馈学习**：利用裁判系统反馈优化切换时机
2. **预测模型**：基于历史数据预测最优切换时间
3. **多目标追踪**：同时追踪所有扇叶，动态选择最优目标
4. **自适应策略**：根据实时状态调整参数
5. **视觉反馈**：在图像上显示当前阶段和目标

## 示例代码

### 完整的buff任务循环示例

```cpp
// 初始化
auto_buff::Buff_Detector detector(config_path);
auto_buff::BigTarget target;  // 或 SmallTarget
auto_buff::Buff_Aimer aimer(config_path);

// 主循环
while (running) {
  cv::Mat img = get_image();
  auto timestamp = std::chrono::steady_clock::now();

  // 使用快速击打模式
  auto powerrune = detector.detect_fast_big_rune(img, 24.0);

  if (powerrune.has_value()) {
    // 更新目标
    target.get_target(powerrune, timestamp);

    // 预测
    target.predict(0.1);

    // 弹道解算
    auto [yaw, pitch] = aimer.get_angle(target, timestamp);

    // 发送给云台
    send_to_gimbal(yaw, pitch);

    // 检查是否完成激活
    if (powerrune->is_big_rune_mode()) {
      auto phase = powerrune->fast_hit_strategy.phase;
      if (phase == auto_buff::BigRuneHitPhase::WAIT_RESET) {
        // 等待下一个周期
      }
    }
  }
}
```

## 联系与反馈

如有问题或建议，请联系开发团队或提交Issue。
