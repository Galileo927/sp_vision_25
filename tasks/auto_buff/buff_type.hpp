#ifndef BUFF__TYPE_HPP
#define BUFF__TYPE_HPP

#include <algorithm>
#include <deque>
#include <eigen3/Eigen/Dense>  // 必须在opencv2/core/eigen.hpp上面
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <optional>
#include <string>
#include <vector>
#include <chrono>

#include "tools/math_tools.hpp"
namespace auto_buff
{
const int INF = 1000000;
enum PowerRune_type { SMALL, BIG };
enum FanBlade_type { _target, _unlight, _light };
enum Track_status { TRACK, TEM_LOSE, LOSE };

/// 大符快速击打状态机
enum class BigRuneHitPhase {
  FIRST_TARGET,    // 准备击打第1个目标
  SECOND_TARGET,   // 准备击打第2个目标
  WAIT_RESET       // 等待重新随机
};

/// 大符快速击打策略
struct BigRuneFastHitStrategy {
  BigRuneHitPhase phase = BigRuneHitPhase::FIRST_TARGET;
  int primary_idx = 0;           // 第1个目标的索引
  int secondary_idx = 1;         // 第2个目标的索引
  std::chrono::steady_clock::time_point phase_start;
  bool switched = false;         // 是否已经切换过

  /// 获取当前应该击打的目标索引
  int get_target_index() const {
    switch (phase) {
      case BigRuneHitPhase::FIRST_TARGET:
        return primary_idx;
      case BigRuneHitPhase::SECOND_TARGET:
        return secondary_idx;
      default:
        return primary_idx;
    }
  }

  /// 更新策略状态
  void update(double estimated_hit_time, double max_second_window = 1.0) {
    if (switched && phase == BigRuneHitPhase::SECOND_TARGET) {
      auto now = std::chrono::steady_clock::now();
      auto elapsed = std::chrono::duration<double>(now - phase_start).count();

      // 超时后进入等待重置状态
      if (elapsed > max_second_window) {
        phase = BigRuneHitPhase::WAIT_RESET;
      }
      return;
    }

    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration<double>(now - phase_start).count();

    switch (phase) {
      case BigRuneHitPhase::FIRST_TARGET:
        // 估计击中时间后切换到第2个目标
        if (elapsed > estimated_hit_time) {
          phase = BigRuneHitPhase::SECOND_TARGET;
          phase_start = now;
          switched = true;
        }
        break;

      case BigRuneHitPhase::SECOND_TARGET:
        // 超时或完成，等待重新随机
        if (elapsed > max_second_window) {
          phase = BigRuneHitPhase::WAIT_RESET;
        }
        break;

      case BigRuneHitPhase::WAIT_RESET:
        // 等待外部调用reset()
        break;
    }
  }

  /// 重置策略（用于新的2扇叶周期）
  void reset() {
    phase = BigRuneHitPhase::FIRST_TARGET;
    phase_start = std::chrono::steady_clock::now();
    switched = false;
  }

  /// 检测是否进入新的2扇叶周期
  bool detect_new_cycle(int current_light_num, int last_light_num) {
    // 当light_num从其他值变为2，或者从2变为其他值再变回2
    return current_light_num == 2 && last_light_num != 2;
  }
};

class FanBlade
{
public:
  cv::Point2f center;               // 扇页中心
  std::vector<cv::Point2f> points;  // 四个点从左上角开始逆时针
  double angle, width, height;
  FanBlade_type type;  // 类型

  explicit FanBlade() = default;

  explicit FanBlade(const std::vector<cv::Point2f> & kpt, cv::Point2f keypoints_center, FanBlade_type t);

  explicit FanBlade(FanBlade_type t);
};

class PowerRune
{
public:
  cv::Point2f r_center;             // R标
  std::vector<FanBlade> fanblades;  // 按target开始顺时针

  int light_num;

  Eigen::Vector3d xyz_in_world;  // 单位：m
  Eigen::Vector3d ypr_in_world;  // 单位：rad
  Eigen::Vector3d ypd_in_world;  // 球坐标系

  Eigen::Vector3d blade_xyz_in_world;  // 单位：m
  Eigen::Vector3d blade_ypd_in_world;  // 球坐标系, 单位: m

  /// 大符快速击打策略状态
  BigRuneFastHitStrategy fast_hit_strategy;

  explicit PowerRune(
    std::vector<FanBlade> & ts, const cv::Point2f r_center,
    std::optional<PowerRune> last_powerrune);
  explicit PowerRune() = default;

  FanBlade & target() { return fanblades[0]; };
  const FanBlade & target() const { return fanblades[0]; };

  bool is_unsolve() const { return unsolvable_; }

  /// 判断是否为大符模式（2个点亮扇叶）
  bool is_big_rune_mode() const { return light_num == 2; }

  /// 获取大符模式下的第2个目标（如果存在）
  const FanBlade * secondary_target() const {
    if (is_big_rune_mode() && fanblades.size() >= 2) {
      return &fanblades[1];
    }
    return nullptr;
  }

private:
  double target_angle_;
  bool unsolvable_ = false;

  double atan_angle(cv::Point2f v) const;  // [0, 2CV_PI]
};
}  // namespace auto_buff
#endif  // BUFF_TYPE_HPP
