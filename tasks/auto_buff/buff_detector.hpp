#ifndef AUTO_BUFF__TRACK_HPP
#define AUTO_BUFF__TRACK_HPP

#include <yaml-cpp/yaml.h>

#include <deque>
#include <optional>

#include "buff_type.hpp"
#include "tools/img_tools.hpp"
#include "yolo11_buff.hpp"
const int LOSE_MAX = 20;  // 丢失的阙值
namespace auto_buff
{
class Buff_Detector
{
public:
  Buff_Detector(const std::string & config);

  std::optional<PowerRune> detect_24(cv::Mat & bgr_img);

  std::optional<PowerRune> detect(cv::Mat & bgr_img);

  std::optional<PowerRune> detect_debug(cv::Mat & bgr_img, cv::Point2f v);

  /// 大符快速连续击打模式
  std::optional<PowerRune> detect_fast_big_rune(cv::Mat & bgr_img, double bullet_speed = 24.0);

private:
  void handle_img(const cv::Mat & bgr_img, cv::Mat & dilated_img);

  cv::Point2f get_r_center(std::vector<FanBlade> & fanblades, cv::Mat & bgr_img);

  void handle_lose();

  /// 选择最佳击打目标（大符双目标模式）
  int select_best_target_for_big_rune(
    const std::vector<YOLO11_BUFF::Object> & results,
    const BigRuneFastHitStrategy & strategy);

  /// 根据旋转方向和角度选择最优目标
  int select_target_by_rotation(
    const std::vector<YOLO11_BUFF::Object> & results,
    int rotation_direction);

  /// 计算估计的击打时间（包括飞行时间和系统延迟）
  double estimate_hit_time(double distance, double bullet_speed);

  YOLO11_BUFF MODE_;
  Track_status status_;
  int lose_;  // 丢失的次数
  double lastlen_;
  std::optional<PowerRune> last_powerrune_ = std::nullopt;

  /// 大符快速击打状态
  int last_light_num_ = 0;  // 上一帧的点亮扇叶数量
};
}  // namespace auto_buff
#endif  // DETECTOR_HPP