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

/// 大符快速连续击打模式实现
std::optional<PowerRune> Buff_Detector::detect_fast_big_rune(cv::Mat & bgr_img, double bullet_speed)
{
  // --------------------------------------------------------
  // 第一步：获取所有 YOLO 候选框
  // --------------------------------------------------------
  std::vector<YOLO11_BUFF::Object> results = MODE_.get_multicandidateboxes(bgr_img);

  if (results.empty()) {
    handle_lose();
    return std::nullopt;
  }

  int current_light_num = results.size();

  // --------------------------------------------------------
  // 第二步：检测大符模式（2个点亮扇叶）
  // --------------------------------------------------------
  bool is_big_rune = (current_light_num == 2);

  if (!is_big_rune) {
    // 小符模式或其他情况，使用原有逻辑
    last_light_num_ = current_light_num;
    return detect(bgr_img);
  }

  // --------------------------------------------------------
  // 第三步：大符快速击打逻辑
  // --------------------------------------------------------
  // 获取或创建策略
  static BigRuneFastHitStrategy strategy_storage;
  BigRuneFastHitStrategy & strategy = last_powerrune_.has_value()
    ? last_powerrune_->fast_hit_strategy
    : strategy_storage;

  // 检测是否进入新的2扇叶周期
  if (strategy.detect_new_cycle(current_light_num, last_light_num_)) {
    strategy.reset();
    tools::logger()->info("[Buff_Detector] 检测到新的大符周期，重置击打策略");
  }

  // 更新策略状态
  double estimated_distance = 0.0;
  if (last_powerrune_.has_value()) {
    estimated_distance = last_powerrune_->ypd_in_world[2];  // 使用上一帧的距离
  } else {
    estimated_distance = 5.0;  // 默认距离估计
  }

  double estimated_hit_time = estimate_hit_time(estimated_distance, bullet_speed);
  strategy.update(estimated_hit_time);

  // --------------------------------------------------------
  // 第四步：根据策略选择目标
  // --------------------------------------------------------
  int target_idx = select_best_target_for_big_rune(results, strategy);

  // --------------------------------------------------------
  // 第五步：构造 PowerRune（包含双目标信息）
  // --------------------------------------------------------
  std::vector<FanBlade> fanblades;

  // 添加所有检测到的扇叶
  for (size_t i = 0; i < results.size(); ++i) {
    FanBlade_type type = (i == target_idx) ? _target : _light;
    fanblades.emplace_back(FanBlade(results[i].kpt, results[i].kpt[4], type));
  }

  // 计算 R 标中心
  auto r_center = get_r_center(fanblades, bgr_img);

  // 构造 PowerRune，保留策略状态
  PowerRune powerrune(fanblades, r_center, last_powerrune_);
  powerrune.fast_hit_strategy = strategy;

  if (powerrune.is_unsolve()) {
    handle_lose();
    return std::nullopt;
  }

  status_ = TRACK;
  lose_ = 0;
  last_light_num_ = current_light_num;

  std::optional<PowerRune> P;
  P.emplace(powerrune);
  last_powerrune_ = P;

  // 调试输出
  tools::logger()->debug("[Buff_Detector] 大符模式 - 阶段: {}, 目标索引: {}/{}",
    static_cast<int>(strategy.phase),
    target_idx,
    results.size() - 1);

  return P;
}

/// 选择最佳击打目标（大符双目标模式）
int Buff_Detector::select_best_target_for_big_rune(
  const std::vector<YOLO11_BUFF::Object> & results,
  const BigRuneFastHitStrategy & strategy)
{
  if (results.size() < 2) {
    return 0;  // 只有一个目标，直接返回
  }

  // 根据策略阶段选择目标
  int desired_idx = strategy.get_target_index();

  // 确保索引有效
  if (desired_idx >= 0 && desired_idx < static_cast<int>(results.size())) {
    return desired_idx;
  }

  // 默认策略：根据旋转方向选择
  int rotation_direction = 1;  // 默认顺时针，可以从 last_powerrune_ 获取
  if (last_powerrune_.has_value()) {
    // 可以从EKF状态获取旋转方向
    double rotation_angle = last_powerrune_->ypr_in_world[2];
    rotation_direction = (rotation_angle > 0) ? 1 : -1;
  }

  return select_target_by_rotation(results, rotation_direction);
}

/// 根据旋转方向和角度选择最优目标
int Buff_Detector::select_target_by_rotation(
  const std::vector<YOLO11_BUFF::Object> & results,
  int rotation_direction)
{
  if (results.size() < 2) {
    return 0;
  }

  // 如果没有R标中心信息，选择离图像中心最近的
  if (!last_powerrune_.has_value()) {
    cv::Point2f img_center(640, 480);  // 假设图像中心
    int best_idx = 0;
    double min_dist = 1e9;

    for (size_t i = 0; i < results.size(); ++i) {
      double dist = cv::norm(results[i].kpt[4] - img_center);
      if (dist < min_dist) {
        min_dist = dist;
        best_idx = i;
      }
    }
    return best_idx;
  }

  // 根据旋转方向选择"上游"或"下游"的目标
  cv::Point2f r_center = last_powerrune_->r_center;

  // 计算每个扇叶相对于R标的角度
  std::vector<std::pair<double, int>> angle_indices;
  for (size_t i = 0; i < results.size(); ++i) {
    cv::Point2f v = results[i].kpt[4] - r_center;
    double angle = std::atan2(v.y, v.x);  // [-PI, PI]
    angle_indices.push_back({angle, i});
  }

  // 按角度排序
  std::sort(angle_indices.begin(), angle_indices.end());

  // 根据旋转方向选择
  // 顺时针：选择角度较大的（下一个到达）
  // 逆时针：选择角度较小的（下一个到达）
  if (rotation_direction > 0) {
    return angle_indices.back().second;  // 最大的角度
  } else {
    return angle_indices.front().second;  // 最小的角度
  }
}

/// 计算估计的击打时间（包括飞行时间和系统延迟）
double Buff_Detector::estimate_hit_time(double distance, double bullet_speed)
{
  // 子弹飞行时间
  double flight_time = distance / bullet_speed;

  // 系统延迟（包括检测、处理、云台响应等）
  const double SYSTEM_DELAY = 0.15;  // 150ms

  // 额外的安全裕量
  const double SAFETY_MARGIN = 0.1;  // 100ms

  return flight_time + SYSTEM_DELAY + SAFETY_MARGIN;
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