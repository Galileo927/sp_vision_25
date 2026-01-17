#include <fmt/core.h>
#include <fmt/format.h>

#include <chrono>
#include <fstream>
#include <iomanip>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

#include "io/camera.hpp"
#include "tasks/auto_buff/buff_aimer.hpp"
#include "tasks/auto_buff/buff_detector.hpp"
#include "tasks/auto_buff/buff_solver.hpp"
#include "tasks/auto_buff/buff_target.hpp"
#include "tasks/auto_buff/buff_type.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

struct BenchmarkStats {
  std::string name;
  double total_time_ms;
  double avg_time_ms;
  double max_time_ms;
  double min_time_ms;
  int count;

  BenchmarkStats(std::string n) : name(n), total_time_ms(0), avg_time_ms(0), max_time_ms(0), min_time_ms(1e9), count(0) {}

  void update(double time_ms) {
    total_time_ms += time_ms;
    max_time_ms = std::max(max_time_ms, time_ms);
    min_time_ms = std::min(min_time_ms, time_ms);
    count++;
    avg_time_ms = total_time_ms / count;
  }

  nlohmann::json to_json() const {
    nlohmann::json j;
    j["name"] = name;
    j["total_time_ms"] = total_time_ms;
    j["avg_time_ms"] = avg_time_ms;
    j["max_time_ms"] = max_time_ms;
    j["min_time_ms"] = min_time_ms;
    j["count"] = count;
    return j;
  }
};

struct BenchmarkResult {
  double fps;
  double avg_latency_ms;
  double max_latency_ms;
  double min_latency_ms;
  int total_frames;
  int detected_frames;
  int missed_frames;

  BenchmarkStats detector;
  BenchmarkStats solver;
  BenchmarkStats target;
  BenchmarkStats aimer;
  BenchmarkStats total;

  BenchmarkResult()
    : fps(0),
      avg_latency_ms(0),
      max_latency_ms(0),
      min_latency_ms(1e9),
      total_frames(0),
      detected_frames(0),
      missed_frames(0),
      detector("Detector"),
      solver("Solver"),
      target("Target"),
      aimer("Aimer"),
      total("Total") {}

  nlohmann::json to_json() const {
    nlohmann::json j;
    j["fps"] = fps;
    j["avg_latency_ms"] = avg_latency_ms;
    j["max_latency_ms"] = max_latency_ms;
    j["min_latency_ms"] = min_latency_ms;
    j["total_frames"] = total_frames;
    j["detected_frames"] = detected_frames;
    j["missed_frames"] = missed_frames;
    j["detection_rate"] = total_frames > 0 ? (double)detected_frames / total_frames : 0;
    j["modules"] = {detector.to_json(), solver.to_json(), target.to_json(), aimer.to_json()};
    return j;
  }

  void print_report() const {
    fmt::print("\n");
    fmt::print("========================================\n");
    fmt::print("       Buff模块性能基准测试报告\n");
    fmt::print("========================================\n");
    fmt::print("\n");
    fmt::print("整体性能:\n");
    fmt::print("  FPS: {:.2f}\n", fps);
    fmt::print("  总帧数: {}\n", total_frames);
    fmt::print("  检测成功: {} ({:.1f}%)\n", detected_frames, 
      total_frames > 0 ? (double)detected_frames / total_frames * 100 : 0);
    fmt::print("  检测失败: {} ({:.1f}%)\n", missed_frames,
      total_frames > 0 ? (double)missed_frames / total_frames * 100 : 0);
    fmt::print("\n");
    fmt::print("延迟统计 (ms):\n");
    fmt::print("  平均延迟: {:.3f}\n", avg_latency_ms);
    fmt::print("  最大延迟: {:.3f}\n", max_latency_ms);
    fmt::print("  最小延迟: {:.3f}\n", min_latency_ms);
    fmt::print("\n");
    fmt::print("各模块耗时统计 (ms):\n");
    fmt::print("  {:<12} {:>10} {:>10} {:>10} {:>10}\n", 
      "模块", "平均", "最大", "最小", "次数");
    fmt::print("  {:<12} {:>10} {:>10} {:>10} {:>10}\n", 
      "----", "----", "----", "----", "----");
    fmt::print("  {:<12} {:>10.3f} {:>10.3f} {:>10.3f} {:>10}\n", 
      detector.name, detector.avg_time_ms, detector.max_time_ms, 
      detector.min_time_ms, detector.count);
    fmt::print("  {:<12} {:>10.3f} {:>10.3f} {:>10.3f} {:>10}\n", 
      solver.name, solver.avg_time_ms, solver.max_time_ms, 
      solver.min_time_ms, solver.count);
    fmt::print("  {:<12} {:>10.3f} {:>10.3f} {:>10.3f} {:>10}\n", 
      target.name, target.avg_time_ms, target.max_time_ms, 
      target.min_time_ms, target.count);
    fmt::print("  {:<12} {:>10.3f} {:>10.3f} {:>10.3f} {:>10}\n", 
      aimer.name, aimer.avg_time_ms, aimer.max_time_ms, 
      aimer.min_time_ms, aimer.count);
    fmt::print("  {:<12} {:>10.3f} {:>10.3f} {:>10.3f} {:>10}\n", 
      total.name, total.avg_time_ms, total.max_time_ms, 
      total.min_time_ms, total.count);
    fmt::print("\n");
    fmt::print("========================================\n");
  }
};

const std::string keys =
  "{help h usage ? |                        | 输出命令行参数说明 }"
  "{@config-path   | configs/sentry.yaml    | yaml配置文件的路径}"
  "{duration d     | 10                     | 测试持续时间(秒) }"
  "{output o       | buff_benchmark_result | 输出文件前缀 }"
  "{video v        |                        | 使用视频文件测试 }"
  "{no-display     | false                  | 不显示图像}";

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }

  auto config_path = cli.get<std::string>(0);
  auto duration = cli.get<double>("duration");
  auto output_prefix = cli.get<std::string>("output");
  auto video_path = cli.get<std::string>("video");
  bool no_display = cli.get<bool>("no-display");

  tools::Exiter exiter;
  BenchmarkResult result;

  std::unique_ptr<io::Camera> camera;
  std::unique_ptr<cv::VideoCapture> video;

  if (!video_path.empty()) {
    video = std::make_unique<cv::VideoCapture>(video_path);
    if (!video->isOpened()) {
      tools::logger()->error("无法打开视频文件: {}", video_path);
      return -1;
    }
    tools::logger()->info("使用视频文件测试: {}", video_path);
  } else {
    camera = std::make_unique<io::Camera>(config_path);
    tools::logger()->info("使用相机测试");
  }

  auto_buff::Buff_Detector detector(config_path);
  auto_buff::Solver solver(config_path);
  auto_buff::SmallTarget target;
  auto_buff::Aimer aimer(config_path);

  cv::Mat img;
  std::chrono::steady_clock::time_point timestamp;
  auto start_time = std::chrono::steady_clock::now();
  auto last_time = start_time;

  tools::logger()->info("开始Buff模块性能基准测试...");
  tools::logger()->info("测试时长: {} 秒", duration);

  while (!exiter.exit()) {
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration<double>(now - start_time).count();

    if (elapsed >= duration) {
      tools::logger()->info("测试完成，耗时 {:.2f} 秒", elapsed);
      break;
    }

    if (video) {
      video->read(img);
      timestamp = now;
    } else {
      camera->read(img, timestamp);
    }

    if (img.empty()) {
      tools::logger()->warn("读取图像失败");
      break;
    }

    auto frame_start = std::chrono::steady_clock::now();

    auto t1 = std::chrono::steady_clock::now();
    auto power_runes = detector.detect(img);
    auto t2 = std::chrono::steady_clock::now();
    double detector_time = std::chrono::duration<double, std::milli>(t2 - t1).count();
    result.detector.update(detector_time);

    if (power_runes.has_value()) {
      result.detected_frames++;

      auto t3 = std::chrono::steady_clock::now();
      solver.solve(power_runes);
      auto t4 = std::chrono::steady_clock::now();
      double solver_time = std::chrono::duration<double, std::milli>(t4 - t3).count();
      result.solver.update(solver_time);

      auto t5 = std::chrono::steady_clock::now();
      target.get_target(power_runes, timestamp);
      auto t6 = std::chrono::steady_clock::now();
      double target_time = std::chrono::duration<double, std::milli>(t6 - t5).count();
      result.target.update(target_time);

      auto t7 = std::chrono::steady_clock::now();
      auto target_copy = target;
      auto command = aimer.aim(target_copy, timestamp, 22, false);
      auto t8 = std::chrono::steady_clock::now();
      double aimer_time = std::chrono::duration<double, std::milli>(t8 - t7).count();
      result.aimer.update(aimer_time);
    } else {
      result.missed_frames++;
    }

    auto frame_end = std::chrono::steady_clock::now();
    double frame_time = std::chrono::duration<double, std::milli>(frame_end - frame_start).count();
    result.total.update(frame_time);

    result.total_frames++;

    auto dt = std::chrono::duration<double>(now - last_time).count();
    if (dt > 0) {
      double current_fps = 1.0 / dt;
      result.fps = result.fps * 0.9 + current_fps * 0.1;
    }
    last_time = now;

    result.avg_latency_ms = result.total.avg_time_ms;
    result.max_latency_ms = result.total.max_time_ms;
    result.min_latency_ms = result.total.min_time_ms;

    if (!no_display && result.total_frames % 30 == 0) {
      tools::logger()->info(
        "帧: {:5d} | FPS: {:.2f} | 延迟: {:.2f}ms | 检测: {:.2f}ms | "
        "解算: {:.2f}ms | 追踪: {:.2f}ms | 瞄准: {:.2f}ms",
        result.total_frames, result.fps, result.avg_latency_ms,
        result.detector.avg_time_ms, result.solver.avg_time_ms,
        result.target.avg_time_ms, result.aimer.avg_time_ms);
    }

    if (!no_display) {
      cv::Mat display = img.clone();
      cv::putText(display, fmt::format("FPS: {:.2f}", result.fps), 
        cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);
      cv::putText(display, fmt::format("Latency: {:.2f}ms", result.avg_latency_ms), 
        cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);
      cv::putText(display, fmt::format("Frames: {}", result.total_frames), 
        cv::Point(10, 90), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);

      if (power_runes.has_value()) {
        auto & p = power_runes.value();
        for (int i = 0; i < 4; i++) {
          tools::draw_point(display, p.target().points[i], {0, 255, 0}, 3);
        }
        tools::draw_point(display, p.target().center, {0, 0, 255}, 5);
        tools::draw_point(display, p.r_center, {255, 0, 0}, 5);
      }

      cv::imshow("Buff Benchmark", display);
      int key = cv::waitKey(1);
      if (key == 'q') break;
    }
  }

  cv::destroyAllWindows();

  result.print_report();

  auto json_output_path = fmt::format("{}.json", output_prefix);
  std::ofstream json_file(json_output_path);
  if (json_file.is_open()) {
    json_file << std::setw(2) << result.to_json() << std::endl;
    json_file.close();
    tools::logger()->info("性能报告已保存到: {}", json_output_path);
  }

  return 0;
}
