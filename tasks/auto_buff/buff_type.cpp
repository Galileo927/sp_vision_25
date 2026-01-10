#include "buff_type.hpp"

#include <algorithm>

#include "tools/logger.hpp"
namespace auto_buff
{
FanBlade::FanBlade(
  const std::vector<cv::Point2f> & kpt, cv::Point2f keypoints_center, FanBlade_type t)
: center(keypoints_center), type(t)
{
  points.insert(points.end(), kpt.begin(), kpt.end());
}

FanBlade::FanBlade(FanBlade_type t) : type(t)
{
  if (t != _unlight) exit(-1);
}

PowerRune::PowerRune(
  std::vector<FanBlade> & ts, const cv::Point2f center, std::optional<PowerRune> last_powerrune)
: r_center(center), light_num(ts.size())
{
  /// 找出target

  // 只有一个fanblade，就为target
  if (light_num == 1) ts[0].type = _target;
  
  // -------------------------------------------------------------
  // 新增逻辑：处理初次识别到多个扇叶（如大符起手亮2个）
  // -------------------------------------------------------------
  else if (!last_powerrune.has_value() && light_num > 1) {
    // 策略：选择离R标中心最近，或者离图像中心最近的（这里ts没有图像信息，取R标角度判定?）
    // 简化策略：直接取第一个作为target，或者取最低点的
    // 或者取距离 r_center 距离最标准的？
    // 这里简单取第一个
    ts[0].type = _target;
  }

  // 没有新亮起来的fanblade (数量一致，保持跟踪)
  else if (last_powerrune.has_value() && ts.size() == last_powerrune.value().light_num) {
    auto last_target_center = last_powerrune.value().fanblades[0].center;
    auto target_fanblade_it = ts.begin();  // 初始化为 fanblades 的第一个元素
    float min_distance = norm(ts[0].center - last_target_center);
    for (auto it = ts.begin(); it != ts.end(); ++it) {
      float distance = norm(it->center - last_target_center);
      if (distance < min_distance) {
        min_distance = distance;
        target_fanblade_it = it;  // 更新最近的 fanblade 的迭代器
      }
    }
    target_fanblade_it->type = _target;  // 设置最近的 fanblade 的 type
    std::iter_swap(ts.begin(), target_fanblade_it);
  }
  // -------------------------------------------------------------
  // 修改逻辑：处理多扇叶情况下的任意切换
  // 如果数量增加了（不管是+1还是+2），或者逻辑不匹配
  // -------------------------------------------------------------
  else if (last_powerrune.has_value() && light_num > last_powerrune.value().light_num) {
    auto last_fanblades = last_powerrune.value().fanblades;
    float max_min_distance = -1.0f;        // 寻找离所有旧目标都很远的新目标
    auto target_fanblade_it = ts.begin();
    
    // 如果是light_num从 0/1 变成 2 (大符双亮)
    // 寻找哪一个是新的？ 其实两个都可能是新的。
    // 但是在 "Hit one, then hit other" 场景:
    // 之前: active A, B. Hit A. A becomes inactive (unlit/diff class).
    // Now: active B. Hit B...
    
    // 这里保留 原逻辑：寻找最“新”的一个（距离旧集合最远）
    for (auto it = ts.begin(); it != ts.end(); ++it) {
      float min_distance = std::numeric_limits<float>::max();
      for (const auto & last_fanblade : last_fanblades) {
        if (last_fanblade.type == _unlight) continue;
        float distance = norm(it->center - last_fanblade.center);
        if (distance < min_distance) {
          min_distance = distance;
        }
      }
      // 如果 last_fanblades 全是 unlight (比如上一帧没识别到 active)，min_dist 还是 max
      if (last_fanblades.size() == 0 || min_distance == std::numeric_limits<float>::max()) {
          min_distance = 10000; // 只要有点距离就行
      }

      if (min_distance > max_min_distance) {
        max_min_distance = min_distance;
        target_fanblade_it = it;
      }
    }
    target_fanblade_it->type = _target;
    std::iter_swap(ts.begin(), target_fanblade_it);
  }
  // Fallback: 如果数量减少了（打灭了一个？），或者其他情况
  // 仍然尝试追踪距离上一帧 Target 最近的一个
  else if (last_powerrune.has_value()) {
     auto last_target_center = last_powerrune.value().fanblades[0].center;
     auto target_fanblade_it = ts.begin();
     float min_distance = norm(ts[0].center - last_target_center);
     for (auto it = ts.begin(); it != ts.end(); ++it) {
        float distance = norm(it->center - last_target_center);
        if (distance < min_distance) {
          min_distance = distance;
          target_fanblade_it = it;
        }
     }
     target_fanblade_it->type = _target;
     std::iter_swap(ts.begin(), target_fanblade_it);
  }
  else {
    tools::logger()->debug("[PowerRune] 识别出错, 无 last_rune 且数量不为1/2?");
    // 强制选第一个防崩
    ts[0].type = _target; 
    // unsolvable_ = true;
    // return;
  }

  /// 填充FanBlade.angle

  double angle = atan_angle(ts[0].center);
  for (auto & t : ts) {
    t.angle = atan_angle(t.center) - angle;
    if (t.angle < -1e-3) t.angle += CV_2PI;
  }

  /// fanblades调整顺序

  std::sort(ts.begin(), ts.end(), [](const FanBlade & a, const FanBlade & b) {
    return a.angle < b.angle;
  });  // 按照 t.angle 从小到大排序 ts
  const std::vector<double> target_angles = {
    0, 2.0 * CV_PI / 5.0, 4.0 * CV_PI / 5.0, 6.0 * CV_PI / 5.0, 8.0 * CV_PI / 5.0};
  for (int i = 0, j = 0; i < 5 && j < ts.size(); i++) {
    if (std::fabs(ts[j].angle - target_angles[i]) < CV_PI / 5.0)
      fanblades.emplace_back(ts[j++]);
    else
      fanblades.emplace_back(FanBlade(_unlight));
  }
};

double PowerRune::atan_angle(cv::Point2f point) const
{
  auto v = point - r_center;
  auto angle = std::atan2(v.y, v.x);
  return angle >= 0 ? angle : angle + CV_2PI;
}
}  // namespace auto_buff
