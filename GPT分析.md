# GPT分析

## 项目总体理解
- 仓库是 Tongji SuperPower 战队的 RoboMaster 视觉框架，单进程独占相机，根据下位机模式在自瞄（auto_aim）与打符（auto_buff）功能间切换，共用 `io::Gimbal` 接口分发姿态与指令。整体结构按 `readme.md` 所述分为 `io/`（硬件抽象）、`tasks/`（算法模块）、`tools/`（公共工具），主程序位于 `src/`。
- 例如 `src/standard_mpc.cpp`：初始化云台、相机、YOLO、规划器等服务，循环读取图像与姿态四元数。若模式为 AUTO_AIM 走装甲板追踪；若切到 `SMALL_BUFF/BIG_BUFF` 则走打符流程，输出 `auto_aim::Plan` 给云台。

## auto_buff 模块流水线
1. **检测** (`Buff_Detector`)
   - 读取配置中的 OpenVINO IR 模型（如 `configs/standard3.yaml` 的 `assets/yolo11_buff_int8.xml`），由 `YOLO11_BUFF` 完成推理，输出 bbox + 6 个关键点。
   - `get_r_center` 结合关键点几何、二值化+膨胀后的 ROI 选出圆心，减少误检；`PowerRune` 根据上一帧状态决定当前点亮扇叶并按 72° 顺序对齐。
2. **位姿解算** (`Solver`)
   - 使用 4 个角点做 IPPE `solvePnP`，通过配置的内参和外参矩阵将 buff 坐标转换到世界坐标，并保存结果供后续预测与重投影调试。
3. **目标跟踪** (`SmallTarget` / `BigTarget`)
   - 小符：7 维 EKF（R 点 yaw/vel、pitch、距离、buff yaw、roll、自旋速度），两步更新分别约束环中心与扇叶位置，同时用 `Voter` 表决旋向。
   - 大符：扩展到 10 维（额外估计 `spd/a/ω/φ`），并用 `tools::RansacSineFitter` 将速度拟合为正弦，以匹配真实运动规律。
4. **瞄准/射击** (`Aimer`)
   - YAML 中的 `yaw_offset/pitch_offset/fire_gap_time/predict_time` 控制零偏、射速节奏与预测视窗。
   - `get_send_angle` 先考虑识别延迟，再用两次弹道解算验证飞行时间，得到目标世界坐标并输出 yaw/pitch。
   - `mpc_aim` 将角度命令封装成 `auto_aim::Plan`：检测到扇叶跳变时暂停射击；正常时计算 yaw/pitch 的速度/加速度前馈；`fire_gap_time` 与 `switch_fanblade_` 确保切换期间不误射。

## 配置与测试
- YAML 同时提供 detector/aimer 参数、相机/云台外参、CAN/串口 ID，保证解算和硬件对齐。
- `tests/auto_buff_test.cpp` 支持离线回放：输入 `.avi + .txt`（姿态），按实机逻辑执行检测→解算→跟踪→瞄准，并借助 Plotter/重投影验证效果，是推荐的调试入口。

## 建议
1. 调参优先修改 `configs/*.yaml`，例如模型路径、零偏、发射节奏，保持公共代码可复用。
2. 改动算法后，先用 `./build/auto_buff_test <数据集> --config-path <yaml>` 离线验证，再上机测试。
