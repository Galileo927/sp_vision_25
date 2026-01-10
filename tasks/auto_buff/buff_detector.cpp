#include "buff_detector.hpp"

#include "tools/logger.hpp"

namespace auto_buff
{
Buff_Detector::Buff_Detector(const std::string & config) : status_(LOSE), lose_(0), MODE_(config) {}

void Buff_Detector::handle_img(const cv::Mat & bgr_img, cv::Mat & dilated_img)
{
  // 彩色图转灰度图
  cv::Mat gray_img;
  cv::cvtColor(bgr_img, gray_img, cv::COLOR_BGR2GRAY);  // 彩色图转灰度图
  // cv::imshow("gray", gray_img);  // 调试用

  // 进行二值化           :把高于100变成255，低于100变成0
  cv::Mat binary_img;
  cv::threshold(gray_img, binary_img, 100, 255, cv::THRESH_BINARY);
  // cv::imshow("binary", binary_img);  // 调试用

  // 膨胀
  cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));  // 使用矩形核
  cv::dilate(binary_img, dilated_img, kernel, cv::Point(-1, -1), 1);
  // cv::imshow("Dilated Image", dilated_img);  // 调试用
}

cv::Point2f Buff_Detector::get_r_center(std::vector<FanBlade> & fanblades, cv::Mat & bgr_img)
{
  /// error
  if (fanblades.empty()) {
    tools::logger()->debug("[Buff_Detector] 无法计算r_center!");
    return {0, 0};
  }

  /// 算出大概位置
  // 注意：这里仅根据传入的扇叶计算一个参考中心
  cv::Point2f r_center_t = {0, 0};
  for (auto & fanblade : fanblades) {
    auto point5 = fanblade.points[4];  // point5是扇叶的中心
    r_center_t += point5;
  }
  r_center_t /= float(fanblades.size());

  /// 处理图片,mask选出大概范围
  cv::Mat dilated_img;
  handle_img(bgr_img, dilated_img);
  
  // 半径估计：基于扇叶尺寸的一个比例，用于在二值图中搜索R标轮廓
  double radius = cv::norm(fanblades[0].points[2] - fanblades[0].center) * 0.8;
  
  // 这里的Mask逻辑被注释了，如果需要更精确的R标搜索，建议恢复并调整
  // cv::Mat mask = cv::Mat::zeros(dilated_img.size(), CV_8U);  // mask
  // circle(mask, r_center_t, radius, cv::Scalar(255), -1);
  // bitwise_and(dilated_img, mask, dilated_img);               // 将遮罩应用于二值化图像

  /// 获取轮廓点,矩阵框筛选
  std::vector<std::vector<cv::Point>> contours;
  auto r_center = r_center_t;
  cv::findContours(
    dilated_img, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);  // external找外部区域
  
  double ratio_1 = 1e9; // INF
  for (auto & it : contours) {
    auto rotated_rect = cv::minAreaRect(it);
    
    // 过滤掉离扇叶太近的轮廓（防止把扇叶自己当成R标）
    bool is_fan = false;
    for (const auto & fb : fanblades) {
      if (cv::norm(rotated_rect.center - fb.center) < radius) {
        is_fan = true;
        break;
      }
    }
    if (is_fan) continue;

    // 形状评分：正方形程度 + 距离中心的权重
    double ratio = rotated_rect.size.height > rotated_rect.size.width
                     ? rotated_rect.size.height / rotated_rect.size.width
                     : rotated_rect.size.width / rotated_rect.size.height;
    ratio += cv::norm(rotated_rect.center - r_center_t) / (radius / 3);
    
    if (ratio < ratio_1) {
      ratio_1 = ratio;
      r_center = rotated_rect.center;
    }
  }
  return r_center;
};

void Buff_Detector::handle_lose()
{
  lose_++;
  if (lose_ >= LOSE_MAX) {
    status_ = LOSE;
    last_powerrune_ = std::nullopt;
  }
  status_ = TEM_LOSE;
}

std::optional<PowerRune> Buff_Detector::detect_24(cv::Mat & bgr_img)
{
  /// onnx 模型检测
  std::vector<YOLO11_BUFF::Object> results = MODE_.get_multicandidateboxes(bgr_img);

  /// 处理未获得的情况
  if (results.empty()) {
    handle_lose();
    return std::nullopt;
  }

  /// results转扇叶FanBlade
  std::vector<FanBlade> fanblades;
  for (auto & result : results) fanblades.emplace_back(FanBlade(result.kpt, result.kpt[4], _light));

  /// 生成PowerRune
  auto r_center = get_r_center(fanblades, bgr_img);
  PowerRune powerrune(fanblades, r_center, last_powerrune_);

  /// handle error
  if (powerrune.is_unsolve()) {
    handle_lose();
    return std::nullopt;
  }

  status_ = TRACK;
  lose_ = 0;
  std::optional<PowerRune> P;
  P.emplace(powerrune);
  last_powerrune_ = P;
  return P;
}

std::optional<PowerRune> Buff_Detector::detect(cv::Mat & bgr_img)
{
  // --------------------------------------------------------
  // 第一步：获取所有 YOLO 候选框
  // --------------------------------------------------------
  std::vector<YOLO11_BUFF::Object> results = MODE_.get_multicandidateboxes(bgr_img);

  if (results.empty()) {
    handle_lose();
    return std::nullopt;
  }

  // --------------------------------------------------------
  // 第二步：目标筛选与锁定策略 (Target Selection & Locking)
  // --------------------------------------------------------
  int best_idx = -1;

  // 策略A：【追踪模式】(TRACK)
  // 如果处于追踪状态且有上一帧的目标记录，优先选择距离上一帧目标最近的那个
  if (status_ == TRACK && last_powerrune_.has_value()) {
    double min_dist = 1e9;
    // 获取上一帧锁定目标的扇叶中心
    cv::Point2f last_center = last_powerrune_->target().center; 

    for (int i = 0; i < results.size(); i++) {
        // 简单过滤：忽略置信度过低的目标
        if (results[i].prob < 0.4) continue;
        
        // 计算距离：当前检测框中心 vs 上一帧目标中心
        // 注意：这里假设 kpt[4] 是扇叶中心点，需与 FanBlade 构造函数一致
        double dist = cv::norm(results[i].kpt[4] - last_center);
        
        // 阈值保护：如果最近的距离也很大（例如 > 200像素），说明可能发生了剧烈跳变或跟丢
        if (dist < min_dist) { // 可以加 && dist < MAX_JUMP_PIXEL
            min_dist = dist;
            best_idx = i;
        }
    }
  }

  // 策略B：【搜索模式】(LOSE) 或 策略A未选中
  // 如果没有锁定目标，或者跟丢了，则选择离图像中心最近的目标（方便云台捕获）
  if (best_idx == -1) {
    double min_dist = 1e9;
    cv::Point2f img_center(bgr_img.cols / 2.0f, bgr_img.rows / 2.0f);

    for (int i = 0; i < results.size(); i++) {
        if (results[i].prob < 0.4) continue;
        
        double dist = cv::norm(results[i].kpt[4] - img_center);
        if (dist < min_dist) {
            min_dist = dist;
            best_idx = i;
        }
    }
  }

  // 如果经过策略A和B还是没找到合适的目标（例如所有目标置信度都低）
  if (best_idx == -1) {
      handle_lose();
      return std::nullopt;
  }

  // --------------------------------------------------------
  // 第三步：构造 PowerRune (仅使用选定的最佳目标)
  // --------------------------------------------------------
  std::vector<FanBlade> fanblades;
  
  // 选中最佳目标
  auto & best_result = results[best_idx];
  fanblades.emplace_back(FanBlade(best_result.kpt, best_result.kpt[4], _light));

  // 计算 R 标中心
  // 注意：get_r_center 会使用传入的 fanblades 计算。
  // 虽然只传入一个扇叶可能降低 R 标定位精度，但保证了 PnP 解算的唯一性和稳定性。
  auto r_center = get_r_center(fanblades, bgr_img);
  
  PowerRune powerrune(fanblades, r_center, last_powerrune_);

  /// handle error
  if (powerrune.is_unsolve()) {
    handle_lose();
    return std::nullopt;
  }

  status_ = TRACK;
  lose_ = 0;
  std::optional<PowerRune> P;
  P.emplace(powerrune);
  last_powerrune_ = P;
  return P;
}

std::optional<PowerRune> Buff_Detector::detect_debug(cv::Mat & bgr_img, cv::Point2f v)
{
  /// onnx 模型检测

  std::vector<YOLO11_BUFF::Object> results = MODE_.get_multicandidateboxes(bgr_img);

  /// 处理未获得的情况

  if (results.empty()) return std::nullopt;

  /// results转扇叶FanBlade

  std::vector<FanBlade> fanblades_t;
  for (auto & result : results)
    fanblades_t.emplace_back(FanBlade(result.kpt, result.kpt[4], _light));

  /// 计算r_center,筛选fanblade
  auto r_center = get_r_center(fanblades_t, bgr_img);
  std::vector<FanBlade> fanblades;
  for (auto & fanblade : fanblades_t) {
    if (cv::norm((fanblade.center - r_center) - v) < 10 || results.size() == 1) {
      fanblades.emplace_back(fanblade);
      break;
    }
  }
  if (fanblades.empty()) return std::nullopt;
  PowerRune powerrune(fanblades, r_center, std::nullopt);

  std::optional<PowerRune> P;
  P.emplace(powerrune);
  return P;
}

}  // namespace auto_buff