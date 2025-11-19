#ifndef TOOLS__SVD_ENHANCED_KALMAN_HPP
#define TOOLS__SVD_ENHANCED_KALMAN_HPP

#include "extended_kalman_filter.hpp"
#include <Eigen/SVD>
#include <deque>
#include <limits>

namespace tools
{

/**
 * @brief SVD增强的扩展卡尔曼滤波器
 *
 * 主要解决以下问题：
 * 1. 病态矩阵求逆导致的数值不稳定
 * 2. 协方差矩阵非正定问题
 * 3. 高维度状态向量的降维处理
 * 4. 测量噪声的自适应估计
 */
class SVDEhancedKalmanFilter : public ExtendedKalmanFilter
{
public:
  // SVD配置参数
  struct SVDConfig {
    double condition_number_threshold = 1e8;    // 条件数阈值
    double regularization_factor = 1e-6;       // 正则化因子
    double min_singular_value_ratio = 1e-10;   // 最小奇异值比率
    double energy_threshold = 0.95;            // 能量保留阈值（降维用）
    bool enable_truncation = true;             // 是否启用截断
    bool enable_adaptive_noise = true;         // 是否启用自适应噪声
    size_t noise_history_size = 50;            // 噪声历史大小
  };

  SVDEhancedKalmanFilter(
    const Eigen::VectorXd & x0, const Eigen::MatrixXd & P0,
    std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> x_add =
      [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) { return a + b; },
    const SVDConfig& config = SVDConfig());

  // 重写更新函数以支持SVD优化
  Eigen::VectorXd update_svd(
    const Eigen::VectorXd & z, const Eigen::MatrixXd & H, const Eigen::MatrixXd & R,
    std::function<Eigen::VectorXd(const Eigen::VectorXd &)> h =
      [](const Eigen::VectorXd & x) { return x; },
    std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> z_subtract =
      [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) { return a - b; });

  // 协方差矩阵SVD分解和重构
  void decompose_covariance();
  void reconstruct_covariance_svd();
  void ensure_positive_definite();

  // 条件数监控
  double get_condition_number(const Eigen::MatrixXd& matrix);
  bool is_well_conditioned(const Eigen::MatrixXd& matrix);

  // 自适应噪声估计
  void update_noise_estimates(const Eigen::VectorXd& residual);
  Eigen::MatrixXd get_adaptive_measurement_noise();

  // 降维处理
  struct ReducedDimensionResult {
    Eigen::MatrixXd U_principal;
    Eigen::VectorXd singular_values;
    int effective_rank;
  };
  ReducedDimensionResult reduce_dimension(const Eigen::MatrixXd& matrix);

  // SVD性能统计
  struct SVDStats {
    int decomposition_count = 0;
    int truncation_count = 0;
    double avg_condition_number = 0.0;
    double max_condition_number = 0.0;
    int ill_conditioned_count = 0;
  };
  const SVDStats& get_svd_stats() const { return svd_stats_; }

private:
  // SVD内部状态
  Eigen::JacobiSVD<Eigen::MatrixXd>* current_svd_;
  Eigen::MatrixXd U_matrix_, V_matrix_;
  Eigen::VectorXd singular_values_;
  SVDConfig config_;
  SVDStats svd_stats_;

  // 噪声估计历史
  std::deque<Eigen::VectorXd> residual_history_;
  Eigen::MatrixXd adaptive_R_;

  // 辅助函数
  Eigen::MatrixXd stable_matrix_inverse(const Eigen::MatrixXd& matrix);
  Eigen::VectorXd truncate_singular_values(const Eigen::VectorXd& singular_values);
  double estimate_optimal_regularization(const Eigen::VectorXd& singular_values);
};

// 实现SVD增强的卡尔曼滤波器
typedef SVDEhancedKalmanFilter SVDEKF;

}  // namespace tools

#endif  // TOOLS__SVD_ENHANCED_KALMAN_HPP