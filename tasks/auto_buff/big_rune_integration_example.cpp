/**
 * 大符快速击打模式集成示例
 *
 * 这个文件展示了如何在实际的buff任务中集成快速击打功能
 */

#include "buff_detector.hpp"
#include "buff_target.hpp"
#include "buff_aimer.hpp"

namespace auto_buff
{

/**
 * 示例1：最小改动集成
 * 只需修改一行代码即可启用快速击打模式
 */
class BuffTask_MinimalChange
{
public:
  BuffTask_MinimalChange(const std::string & config)
  : detector_(config), target_(BIG), aimer_(config)
  {
  }

  void run(cv::Mat & img)
  {
    auto timestamp = std::chrono::steady_clock::now();

    // ==================== 关键修改 ====================
    // 原代码：auto powerrune = detector_.detect(img);
    // 新代码：
    auto powerrune = detector_.detect_fast_big_rune(img, 24.0);
    // =================================================

    if (powerrune.has_value()) {
      target_.get_target(powerrune, timestamp);

      if (!target_.is_unsolve()) {
        target_.predict(0.1);

        auto angles = aimer_.get_angle(target_, timestamp, true);

        // 发送给云台...
      }
    }
  }

private:
  Buff_Detector detector_;
  BigTarget target_;
  Buff_Aimer aimer_;
};

/**
 * 示例2：带监控的集成
 * 添加性能监控和调试信息
 */
class BuffTask_WithMonitoring
{
public:
  struct PerformanceMetrics {
    int total_groups = 0;
    double total_time = 0.0;
    double avg_group_time = 0.0;
    int first_target_hits = 0;
    int second_target_hits = 0;
  };

  BuffTask_WithMonitoring(const std::string & config)
  : detector_(config), target_(BIG), aimer_(config)
  {
  }

  void run(cv::Mat & img)
  {
    auto timestamp = std::chrono::steady_clock::now();

    auto powerrune = detector_.detect_fast_big_rune(img, 24.0);

    if (powerrune.has_value()) {
      // 监控当前阶段
      auto phase = powerrune->fast_hit_strategy.phase;
      update_metrics(phase);

      // 调试输出
      if (phase == BigRuneHitPhase::SECOND_TARGET) {
        tools::logger()->info("切换到第2个目标");
      }

      target_.get_target(powerrune, timestamp);

      if (!target_.is_unsolve()) {
        target_.predict(0.1);
        auto angles = aimer_.get_angle(target_, timestamp, true);

        // 发送给云台...
      }
    }
  }

  PerformanceMetrics get_metrics() const { return metrics_; }

private:
  Buff_Detector detector_;
  BigTarget target_;
  Buff_Aimer aimer_;
  PerformanceMetrics metrics_;

  void update_metrics(BigRuneHitPhase phase)
  {
    if (phase == BigRuneHitPhase::FIRST_TARGET) {
      metrics_.first_target_hits++;
    } else if (phase == BigRuneHitPhase::SECOND_TARGET) {
      metrics_.second_target_hits++;
    }
  }
};

/**
 * 示例3：自适应参数调整
 * 根据击打效果动态调整参数
 */
class BuffTask_Adaptive
{
public:
  BuffTask_Adaptive(const std::string & config)
  : detector_(config), target_(BIG), aimer_(config),
    bullet_speed_(24.0), switching_factor_(1.0)
  {
  }

  void run(cv::Mat & img)
  {
    auto timestamp = std::chrono::steady_clock::now();

    auto powerrune = detector_.detect_fast_big_rune(
      img,
      bullet_speed_
    );

    if (powerrune.has_value()) {
      auto phase = powerrune->fast_hit_strategy.phase;

      target_.get_target(powerrune, timestamp);

      if (!target_.is_unsolve()) {
        target_.predict(0.1);
        auto angles = aimer_.get_angle(target_, timestamp, true);

        // 模拟击打反馈（实际应该从裁判系统获取）
        bool hit_successful = check_hit_feedback();

        if (hit_successful) {
          on_hit_successful(phase);
        } else {
          on_hit_failed(phase);
        }

        // 发送给云台...
      }
    }
  }

  void on_hit_successful(BigRuneHitPhase phase)
  {
    if (phase == BigRuneHitPhase::FIRST_TARGET) {
      // 第1个目标击中，略微提前切换
      switching_factor_ *= 0.98;
      tools::logger()->debug("第1目标命中，切换因子: {:.3f}", switching_factor_);
    } else if (phase == BigRuneHitPhase::SECOND_TARGET) {
      // 第2个目标也击中，保持当前策略
      tools::logger()->debug("第2目标命中");
    }
  }

  void on_hit_failed(BigRuneHitPhase phase)
  {
    if (phase == BigRuneHitPhase::FIRST_TARGET) {
      // 第1个目标未中，延后切换
      switching_factor_ *= 1.02;
      tools::logger()->debug("第1目标未中，切换因子: {:.3f}", switching_factor_);
    }
    // 第2个目标未中不影响策略
  }

private:
  Buff_Detector detector_;
  BigTarget target_;
  Buff_Aimer aimer_;
  double bullet_speed_;
  double switching_factor_;

  bool check_hit_feedback()
  {
    // TODO: 从裁判系统获取实际击打反馈
    // 目前返回true作为示例
    return true;
  }
};

/**
 * 示例4：自动模式切换
 * 根据light_num自动选择大符或小符模式
 */
class BuffTask_AutoMode
{
public:
  enum class Mode {
    SMALL,  // 小符模式
    BIG     // 大符模式
  };

  BuffTask_AutoMode(const std::string & config)
  : detector_(config),
    small_target_(SMALL),
    big_target_(BIG),
    aimer_(config),
    current_mode_(Mode::SMALL)
  {
  }

  void run(cv::Mat & img)
  {
    auto timestamp = std::chrono::steady_clock::now();

    // 自动检测并切换模式
    auto powerrune = detector_.detect_fast_big_rune(img, 24.0);

    if (powerrune.has_value()) {
      // 检测模式变化
      Mode new_mode = powerrune->is_big_rune_mode() ? Mode::BIG : Mode::SMALL;

      if (new_mode != current_mode_) {
        on_mode_change(new_mode);
        current_mode_ = new_mode;
      }

      // 根据模式选择目标
      if (current_mode_ == Mode::BIG) {
        big_target_.get_target(powerrune, timestamp);

        if (!big_target_.is_unsolve()) {
          big_target_.predict(0.1);
          auto angles = aimer_.get_angle(big_target_, timestamp, true);
          // 发送给云台...
        }
      } else {
        small_target_.get_target(powerrune, timestamp);

        if (!small_target_.is_unsolve()) {
          small_target_.predict(0.1);
          auto angles = aimer_.get_angle(small_target_, timestamp, true);
          // 发送给云台...
        }
      }
    }
  }

private:
  Buff_Detector detector_;
  SmallTarget small_target_;
  BigTarget big_target_;
  Buff_Aimer aimer_;
  Mode current_mode_;

  void on_mode_change(Mode new_mode)
  {
    if (new_mode == Mode::BIG) {
      tools::logger()->info("切换到大符模式");
    } else {
      tools::logger()->info("切换到小符模式");
    }
  }
};

/**
 * 示例5：完整的生产环境实现
 * 包含错误处理、日志记录、性能监控等
 */
class BuffTask_Production
{
public:
  BuffTask_Production(const std::string & config)
  : detector_(config),
    target_(BIG),
    aimer_(config),
    frame_count_(0),
    last_group_time_(std::chrono::steady_clock::now())
  {
    // 从配置文件读取参数
    load_config(config);
  }

  void run(cv::Mat & img)
  {
    frame_count_++;
    auto timestamp = std::chrono::steady_clock::now();

    try {
      auto powerrune = detector_.detect_fast_big_rune(img, config_.bullet_speed);

      if (powerrune.has_value()) {
        update_statistics(*powerrune);

        target_.get_target(powerrune, timestamp);

        if (!target_.is_unsolve()) {
          target_.predict(config_.predict_time);

          auto angles = aimer_.get_angle(target_, timestamp, config_.to_now);

          // 发送给云台
          send_to_gimbal(angles.yaw, angles.pitch);

          // 检查是否完成一组
          check_group_completion(*powerrune);
        } else {
          tools::logger()->warn("Target unsolvable");
        }
      } else {
        tools::logger()->debug("No powerrune detected");
      }

    } catch (const std::exception & e) {
      tools::logger()->error("Error in buff task: {}", e.what());
    }
  }

  void print_statistics()
  {
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration<double>(now - start_time_).count();

    tools::logger()->info("========== Buff Task Statistics ==========");
    tools::logger()->info("运行时间: {:.1f}秒", elapsed);
    tools::logger()->info("总帧数: {}", frame_count_);
    tools::logger()->info("FPS: {:.1f}", frame_count_ / elapsed);
    tools::logger()->info("完成组数: {}", stats_.completed_groups);
    tools::logger()->info("第1目标命中: {}", stats_.first_target_hits);
    tools::logger()->info("第2目标命中: {}", stats_.second_target_hits);
    tools::logger()->info("平均组时间: {:.2f}秒", stats_.avg_group_time);
    tools::logger()->info("=========================================");
  }

private:
  struct Config {
    double bullet_speed = 24.0;
    double predict_time = 0.1;
    bool to_now = true;
  };

  struct Statistics {
    int completed_groups = 0;
    int first_target_hits = 0;
    int second_target_hits = 0;
    double avg_group_time = 0.0;
    double total_group_time = 0.0;
  };

  Buff_Detector detector_;
  BigTarget target_;
  Buff_Aimer aimer_;
  Config config_;
  Statistics stats_;

  int frame_count_;
  std::chrono::steady_clock::time_point start_time_;
  std::chrono::steady_clock::time_point last_group_time_;

  void load_config(const std::string & config_path)
  {
    // TODO: 从YAML配置文件加载参数
    start_time_ = std::chrono::steady_clock::now();
  }

  void update_statistics(const PowerRune & powerrune)
  {
    if (powerrune.is_big_rune_mode()) {
      auto phase = powerrune.fast_hit_strategy.phase;

      if (phase == BigRuneHitPhase::FIRST_TARGET) {
        stats_.first_target_hits++;
      } else if (phase == BigRuneHitPhase::SECOND_TARGET) {
        stats_.second_target_hits++;
      }
    }
  }

  void check_group_completion(const PowerRune & powerrune)
  {
    if (powerrune.is_big_rune_mode()) {
      auto phase = powerrune.fast_hit_strategy.phase;

      if (phase == BigRuneHitPhase::WAIT_RESET) {
        auto now = std::chrono::steady_clock::now();
        auto group_time = std::chrono::duration<double>(
          now - last_group_time_
        ).count();

        stats_.completed_groups++;
        stats_.total_group_time += group_time;
        stats_.avg_group_time =
          stats_.total_group_time / stats_.completed_groups;

        tools::logger()->info(
          "完成第{}组，用时{:.2f}秒，平均{:.2f}秒",
          stats_.completed_groups,
          group_time,
          stats_.avg_group_time
        );

        last_group_time_ = now;
      }
    }
  }

  void send_to_gimbal(double yaw, double pitch)
  {
    // TODO: 实现云台通信
  }
};

} // namespace auto_buff

/**
 * 使用示例
 */
int main()
{
  std::string config_path = "config/buff_config.yaml";

  // 选择一个实现
  auto_buff::BuffTask_Production task(config_path);

  // 模拟主循环
  while (true) {
    cv::Mat img = /* 获取图像 */;
    task.run(img);

    // 每100帧打印统计信息
    if (/* frame_count % 100 == 0 */ false) {
      task.print_statistics();
    }
  }

  return 0;
}
