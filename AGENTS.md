# Repository Guidelines

## Project Structure & Module Organization
- `src/` 提供主程序入口（`standard`, `uav`, 调试版本），通过 YAML 配置组合 IO、识别、规划模块。
- `tasks/auto_aim`, `tasks/auto_buff`, `tasks/omniperception` 是核心业务模块；新增功能时保持公共头文件的接口稳定，调参前先查 `configs/` 内对应 YAML。
- `io/` 管理相机、CAN 板、模式切换等硬件交互，依赖 `configs/` 中的参数。
- `tools/` 存放通用工具（日志、轨迹、EKF、线程）；优先复用这些组件。
- `tests/` 下的可执行文件用于单模块诊断，可直接在 `build/` 目录运行；参数说明记录在源码顶部。
- 标定相关程序与数据位于 `calibration/`、`assets/`，避免手工改动生成文件。
- 所有运行时参数集中在 `configs/`，按场景拆分；创建新配置时继承现有文件，注明硬件与日期。

## Build, Test, and Development Commands
- `cmake -B build -DCMAKE_BUILD_TYPE=Release`：第一次配置工程前先安装 OpenVINO、Ceres、OpenCV 与相机 SDK。
- `cmake --build build -j$(nproc)`：增量编译所有目标和诊断程序；必要时加 `--target standard` 仅生成指定二进制。
- `./build/standard configs/standard3.yaml`：启动默认自瞄流程，需连接相机与下位机。
- `./build/planner_test_offline configs/auto_aim/planner.yaml --d 3.0 --w 5.0`：离线复现规划，调节距离和角速度验证轨迹。
- `./build/auto_aim_test assets/demo/demo --config-path configs/demo.yaml`：回放示例数据，检验识别与跟踪改动。
- `./build/camera_test configs/camera.yaml`：在更新驱动或更换相机后确认曝光、白平衡配置。
- `ctest --test-dir build`：若后续添加单元测试模块，请在 CI 与本地保持该命令通过。

## Coding Style & Naming Conventions
- 使用 C++17，遵循仓库内 `.clang-format`（Google 风格、100 列、指针对齐中置）；提交前执行 `clang-format -i`，并确保 `clang-tidy` 警告已处理。
- 文件与变量使用 snake_case，类型使用 PascalCase；保持命名与模块职责一致，模板参数采用大写单词。
- YAML 配置统一小写加下划线，按模块拆分文件并附简短注释；大型调参请记录默认值和调试场景。
- 输出日志统一调用 `tools::logger()`，按 info/debug 区分运行级别，避免 `std::cout`，高频日志请加节流或条件。

## Testing Guidelines
- 编译后，从 `build/` 目录运行对应测试；图形输出依赖终端交互，请勿后台运行。
- 优先执行离线测试（`planner_test_offline`, `auto_aim_test`）验证算法，再切换至硬件联调。
- 修改硬件接口时，使用相应诊断程序（如 `camera_test`, `gimbal_test`）确认链路，并记录所需设备。
- 规划或控制策略改动需保存 Plotter 曲线或录屏，便于回归比对。
- 目标：自研模块保持至少一次离线回放通过，关键 PR 提交前在机器人上完成实跑或模拟验证。

## Commit & Pull Request Guidelines
- 参考现有短句式提交信息（如 `Fix CMake ROS check`），首行使用祈使语并控制在约 72 字符。
- 如关联 issue，在首行注明编号（示例：`auto_aim: tune planner latency (#123)`），正文说明关键改动与参数调整。
- PR 描述需列出运行过的测试命令、硬件环境、可视化结果；涉及 YAML 变更要指出受影响的场景。
- 审查前请自查是否包含必要截图、日志链接、以及针对回归风险的缓解说明。

## Deployment & Configuration Notes
- 部署脚本（`autostart.sh`、`.desktop`）保持可执行权限，`Exec` 必须填写绝对路径，修改后实机验证。
- 标定产物统一存放在 `configs/` 或 `calibration/record/`，文件名包含设备或时间戳，避免提交密钥或个人账号。
- 启用 GPU 推理时记录 OpenVINO 版本与额外依赖，使用脚本化安装步骤方便复现。
- 远程部署建议通过 `screen` 或 `tmux` 管理会话，确保重启后 `autostart.sh` 能恢复预期服务。
- 发布前备份关键 YAML 配置，保留原始参数便于回滚。
