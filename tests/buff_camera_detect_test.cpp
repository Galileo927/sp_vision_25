#include <fmt/core.h>

#include <chrono>
#include <opencv2/opencv.hpp>

#include "io/camera.hpp"
#include "tasks/auto_buff/buff_detector.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

const std::string keys =
  "{help h usage ? |                        | 输出命令行参数说明 }"
  "{@config-path   | configs/sentry.yaml    | yaml配置文件的路径}";

int main(int argc, char * argv[])
{
  // 读取命令行参数
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  auto config_path = cli.get<std::string>(0);

  tools::Exiter exiter;

  io::Camera camera(config_path);
  auto_buff::Buff_Detector detector(config_path);

  std::chrono::steady_clock::time_point timestamp;

  while (!exiter.exit()) {
    cv::Mat img;
    camera.read(img, timestamp);

    if (img.empty()) break;

    auto last = std::chrono::steady_clock::now();

    // -------- buff检测核心逻辑 --------
    auto power_runes = detector.detect(img);

    auto now = std::chrono::steady_clock::now();
    auto dt = tools::delta_time(now, last);
    tools::logger()->info("{:.2f} fps", 1 / dt);

    // -------- 可视化检测结果 --------
    if (power_runes.has_value()) {
      const auto & p = power_runes.value();
      
      // 绘制检测到的目标点
      for (int i = 0; i < 4; i++) {
        tools::draw_point(img, p.target().points[i], {0, 255, 0}, 3);
      }
      
      // 绘制目标中心
      tools::draw_point(img, p.target().center, {0, 0, 255}, 5);
      
      // 绘制旋转中心
      tools::draw_point(img, p.r_center, {255, 0, 0}, 5);
      
      // 输出检测信息
      tools::logger()->info("Power rune detected!");
      tools::logger()->info("  Center: ({:.2f}, {:.2f})", 
        p.target().center.x, p.target().center.y);
      tools::logger()->info("  Rune Center: ({:.2f}, {:.2f})", 
        p.r_center.x, p.r_center.y);
    } else {
      tools::logger()->info("No power rune detected");
    }

    cv::imshow("Buff Detection Result", img);

    auto key = cv::waitKey(33);
    if (key == 'q') break;
  }

  cv::destroyAllWindows();
  return 0;
}
