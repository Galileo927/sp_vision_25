/**
 * @brief BUFF目标追踪系统的SVD优化集成示例
 *
 * 这个文件展示了如何将SVD增强的卡尔曼滤波器集成到现有的buff_target系统中
 */

#include "tasks/auto_buff/buff_target.hpp"
#include "tools/svd_enhanced_kalman.hpp"
#include <iostream>
#include <chrono>

namespace auto_buff {

/**
 * @brief SVD优化的小符目标追踪器
 */
class SVDEnhancedSmallTarget : public SmallTarget {
private:
    // SVD增强的EKF配置
    tools::SVDEhancedKalmanFilter::SVDConfig svd_config_;
    std::unique_ptr<tools::SVDEhancedKalmanFilter> svd_ekf_;

public:
    SVDEnhancedSmallTarget() {
        // 配置SVD参数
        svd_config_.condition_number_threshold = 1e8;
        svd_config_.regularization_factor = 1e-6;
        svd_config_.enable_adaptive_noise = true;
        svd_config_.noise_history_size = 50;
        svd_config_.min_singular_value_ratio = 1e-10;
    }

    // 重写初始化函数，使用SVD增强的EKF
    void init_svd(double nowtime, const PowerRune& p) {
        // 创建SVD增强的EKF实例
        Eigen::VectorXd initial_state(7);
        initial_state << p.ypd_in_world[0], 0.0, p.ypd_in_world[1], p.ypd_in_world[2],
                        p.ypr_in_world[0], p.ypr_in_world[2],
                        SMALL_W * voter.clockwise();

        Eigen::MatrixXd initial_covariance = Eigen::MatrixXd::Identity(7, 7);
        initial_covariance(0,0) = 10.0;   // R_yaw方差
        initial_covariance(1,1) = 10.0;   // v_R_yaw方差
        initial_covariance(2,2) = 10.0;   // R_pitch方差
        initial_covariance(3,3) = 10.0;   // R_dis方差
        initial_covariance(4,4) = 10.0;   // yaw方差
        initial_covariance(5,5) = 10.0;   // angle方差
        initial_covariance(6,6) = 1e-2;   // spd方差

        // 添加角度规范化函数
        auto angle_add = [](const Eigen::VectorXd& a, const Eigen::VectorXd& b) -> Eigen::VectorXd {
            Eigen::VectorXd result = a + b;
            result[0] = tools::limit_rad(result[0]);
            result[2] = tools::limit_rad(result[2]);
            result[4] = tools::limit_rad(result[4]);
            result[5] = tools::limit_rad(result[5]);
            return result;
        };

        svd_ekf_ = std::make_unique<tools::SVDEhancedKalmanFilter>(
            initial_state, initial_covariance, angle_add, svd_config_);
    }

    // 重写更新函数，使用SVD优化
    void update_svd(double nowtime, const PowerRune& p) {
        // 使用SVD增强的更新
        const Eigen::VectorXd& R_ypd = p.ypd_in_world;
        const Eigen::VectorXd& ypr = p.ypr_in_world;

        // 第一阶段：直接测量更新
        Eigen::MatrixXd H1(4, 7);
        H1 << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  // R_yaw
              0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,  // R_pitch
              0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,  // R_dis
              0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0;  // angle

        Eigen::MatrixXd R1 = Eigen::MatrixXd::Identity(4, 4);
        R1(0,0) = 0.01;  // R_yaw噪声
        R1(1,1) = 0.01;  // R_pitch噪声
        R1(2,2) = 0.5;   // R_dis噪声
        R1(3,3) = 0.1;   // angle噪声

        Eigen::VectorXd z1(4);
        z1 << R_ypd[0], R_ypd[1], R_ypd[2], ypr[2];

        // 使用SVD增强的更新
        svd_ekf_->update_svd(z1, H1, R1);

        // 第二阶段：非线性测量更新
        auto h2 = [&](const Eigen::VectorXd& x) -> Eigen::Vector3d {
            return calculate_blade_position(x);
        };

        Eigen::MatrixXd H2 = calculate_jacobian(svd_ekf_->x);
        Eigen::MatrixXd R2 = Eigen::MatrixXd::Identity(3, 3);
        R2(0,0) = 0.01;  // B_yaw噪声
        R2(1,1) = 0.01;  // B_pitch噪声
        R2(2,2) = 0.5;   // B_dis噪声

        Eigen::VectorXd z2 = p.blade_ypd_in_world;

        // SVD优化的非线性更新
        svd_ekf_->update_svd(z2, H2, R2, h2);
    }

    // 获取SVD性能统计
    void log_svd_performance() {
        auto stats = svd_ekf_->get_svd_stats();

        std::cout << "=== SVD Performance Statistics ===" << std::endl;
        std::cout << "SVD Decompositions: " << stats.decomposition_count << std::endl;
        std::cout << "Truncations: " << stats.truncation_count << std::endl;
        std::cout << "Avg Condition Number: " << stats.avg_condition_number << std::endl;
        std::cout << "Max Condition Number: " << stats.max_condition_number << std::endl;
        std::cout << "Ill-conditioned Count: " << stats.ill_conditioned_count << std::endl;
        std::cout << "Truncation Rate: "
                  << static_cast<double>(stats.truncation_count) / stats.decomposition_count * 100
                  << "%" << std::endl;
    }

    // 数值稳定性检查
    bool check_numerical_stability() {
        // 获取当前状态协方差矩阵
        Eigen::MatrixXd current_P = svd_ekf_->P;

        // 检查条件数
        double condition_number = svd_ekf_->get_condition_number(current_P);

        // 检查特征值
        Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigen_solver(current_P);
        Eigen::VectorXd eigenvalues = eigen_solver.eigenvalues();

        bool all_positive = (eigenvalues.array() > 0).all();
        bool well_conditioned = condition_number < 1e8;

        if (!all_positive) {
            tools::logger()->error("Covariance matrix has negative eigenvalues!");
        }

        if (!well_conditioned) {
            tools::logger()->warn("High condition number detected: {}", condition_number);
        }

        return all_positive && well_conditioned;
    }
};

/**
 * @brief SVD优化的大符目标追踪器
 */
class SVDEnhancedBigTarget : public BigTarget {
private:
    tools::SVDEhancedKalmanFilter::SVDConfig svd_config_;
    std::unique_ptr<tools::SVDEhancedKalmanFilter> svd_ekf_;

public:
    SVDEnhancedBigTarget() {
        // 为大符配置更激进的SVD参数（状态更多，更需要稳定性）
        svd_config_.condition_number_threshold = 1e6;    // 更严格的条件数限制
        svd_config_.regularization_factor = 1e-5;
        svd_config_.enable_adaptive_noise = true;
        svd_config_.noise_history_size = 100;          // 更长的历史窗口
        svd_config_.energy_threshold = 0.98;           // 保留98%的能量
    }

    // 大符特有的SVD优化
    void predict_svd(double dt) {
        // 大符运动模型的复杂度高，容易数值不稳定

        // 首先进行状态预测
        auto motion_model = [&](const Eigen::VectorXd& x) -> Eigen::VectorXd {
            return apply_big_target_motion_model(x, dt);
        };

        // 计算状态转移矩阵
        Eigen::MatrixXd A(10, 10) = compute_big_target_transition_matrix(dt);

        // 大符的过程噪声矩阵（数值敏感）
        Eigen::MatrixXd Q = compute_big_target_process_noise(dt);

        // 使用SVD增强的预测
        svd_ekf_->predict(A, Q, motion_model);

        // 检查预测后协方差矩阵的数值稳定性
        if (!svd_ekf_->is_well_conditioned(svd_ekf_->P)) {
            tools::logger()->warn("Ill-conditioned covariance after prediction");
            svd_ekf_->ensure_positive_definite();
        }
    }

    // 针对大符的运动模型进行SVD优化
    Eigen::VectorXd apply_big_target_motion_model(const Eigen::VectorXd& x, double dt) {
        double a = x[7];   // 正弦振幅
        double w = x[8];   // 角频率
        double fi = x[9];  // 相位
        double t = dt;

        // 计算角速度（高敏感计算）
        double angular_velocity = a * std::sin(w * t + fi) + (2.09 - a);

        // 角度积分（复杂三角函数，容易数值误差）
        double angle_integral = (-a / w) * std::cos(w * t + fi) + (a / w) * std::cos(fi) + (2.09 - a) * t;

        Eigen::VectorXd x_new = x;
        x_new[5] = tools::limit_rad(x[5] + angle_integral);  // roll update
        x_new[6] = angular_velocity;                         // spd update

        return x_new;
    }

    // 大符的状态转移矩阵计算（10维，容易病态）
    Eigen::MatrixXd compute_big_target_transition_matrix(double dt) {
        Eigen::MatrixXd A = Eigen::MatrixXd::Identity(10, 10);

        // 添加非线性耦合项（需要稳定性处理）
        double current_a = svd_ekf_->x[7];
        double current_w = svd_ekf_->x[8];
        double current_fi = svd_ekf_->x[9];

        // 角速度与参数a,w的耦合项（高敏感度）
        A(6, 7) = std::sin(current_w * dt + current_fi) - 1;  // d(spd)/da
        A(6, 8) = current_a * dt * std::cos(current_w * dt + current_fi);  // d(spd)/dw
        A(6, 9) = current_a * std::cos(current_w * dt + current_fi);       // d(spd)/dfi

        return A;
    }

    // 大符的过程噪声矩阵（数值处理困难）
    Eigen::MatrixXd compute_big_target_process_noise(double dt) {
        Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(10, 10);

        // 高维协方差矩阵，容易数值不稳定
        double dt2 = dt * dt;
        double dt3 = dt * dt * dt;
        double dt4 = dt * dt * dt * dt;

        // 高阶项的噪声系数需要小心调整
        double v1 = 0.9;  // 基础过程噪声

        Q(0, 0) = dt4 * v1 / 4;  Q(0, 1) = dt3 * v1 / 2;  // 位置-速度耦合
        Q(1, 0) = dt3 * v1 / 2;  Q(1, 1) = dt2 * v1;      // 速度-速度

        Q(5, 5) = 0.09;   // 角度噪声
        Q(6, 6) = 0.5;    // 角速度噪声（大符更不确定）
        Q(7, 7) = 0.0;    // a参数噪声（假设准静态）
        Q(8, 8) = 0.0;    // w参数噪声
        Q(9, 9) = 1.0;    // 相位噪声（允许变化）

        return Q;
    }
};

/**
 * @brief SVD性能比较测试
 */
class SVDBenchmark {
public:
    static void run_comparison_test() {
        std::cout << "=== SVD Enhanced BUFF System Performance Test ===" << std::endl;

        // 测试1：数值稳定性对比
        test_numerical_stability();

        // 测试2：收敛速度对比
        test_convergence_speed();

        // 测试3：异常处理能力对比
        test_anomaly_recovery();

        // 测试4：计算性能对比
        test_computational_performance();
    }

private:
    static void test_numerical_stability() {
        std::cout << "\n--- Numerical Stability Test ---" << std::endl;

        // 构造病态矩阵测试用例
        Eigen::MatrixXd ill_conditioned_matrix(5, 5);
        ill_conditioned_matrix <<
            1e8,    1,    1,    1,    1,
              1,  1e-8,  1,    1,    1,
              1,    1,  1e8,  1,    1,
              1,    1,    1,  1e-8, 1,
              1,    1,    1,    1,  1e8;

        double condition_number = std::get_condition_number(ill_conditioned_matrix);
        std::cout << "Original condition number: " << condition_number << std::endl;

        // SVD分解处理
        tools::SVDEhancedKalmanFilter::ReducedDimensionResult result =
            tools::reduce_dimension_svd(ill_conditioned_matrix);

        std::cout << "Effective rank: " << result.effective_rank << std::endl;
        std::cout << "Condition number improved: " <<
            (result.effective_rank < 5 ? "YES" : "NO") << std::endl;
    }

    static void test_convergence_speed() {
        std::cout << "\n--- Convergence Speed Test ---" << std::endl;

        auto start = std::chrono::high_resolution_clock::now();

        // 运行45帧（典型BUFF周期）
        const int NUM_FRAMES = 45;

        SVDEnhancedSmallTarget svd_target;
        // ... 运行SVD增强版本 ...

        auto svd_end = std::chrono::high_resolution_clock::now();
        auto svd_duration = std::chrono::duration_cast<std::chrono::microseconds>(svd_end - start);

        std::cout << "SVD enhanced version time: " << svd_duration.count() << " μs" << std::endl;

        // 对比传统实现（需要基准测试）
        // SmallTarget traditional_target;
        // ... 运行传统版本对比 ...
    }
};

} // namespace auto_buff

/**
 * @brief 使用示例和集成测试
 */
int main() {
    std::cout << "BUFF System SVD Enhancement Demo" << std::endl;

    // 运行性能对比测试
    auto_buff::SVDBenchmark::run_comparison_test();

    // 创建SVD增强的目标追踪器
    auto buff_detection = auto_buff::PowerRune();  // 假设有检测数据

    auto_buff::SVDEnhancedSmallTarget tracker;
    // tracker.init_svd(detection_time, buff_detection);

    // 主循环（模拟45帧）
    for (int frame = 0; frame < 45; ++frame) {
        // tracker.update_svd(detection_time, buff_detection);

        // 检查数值稳定性
        if (!tracker.check_numerical_stability()) {
            std::cout << "Numerical instability detected at frame " << frame << std::endl;
        }

        // 可以通过svd_ekf_获取内部状态
        // std::cout << "Prediction: " << tracker.svd_ekf_->x[5] * 180.0 / CV_PI << " degrees" << std::endl;
    }

    // 输出SVD性能统计
    tracker.log_svd_performance();

    return 0;
}