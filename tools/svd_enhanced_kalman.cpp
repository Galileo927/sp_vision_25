#include "svd_enhanced_kalman.hpp"
#include <cmath>
#include <algorithm>

namespace tools {

SVDEhancedKalmanFilter::SVDEhancedKalmanFilter(
  const Eigen::VectorXd & x0, const Eigen::MatrixXd & P0,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> x_add,
  const SVDConfig& config)
  : ExtendedKalmanFilter(x0, P0, x_add), config_(config),
    current_svd_(nullptr), adaptive_R_(Eigen::MatrixXd::Zero(x0.size(), x0.size())){};

Eigen::VectorXd SVDEhancedKalmanFilter::update_svd(
  const Eigen::VectorXd & z, const Eigen::MatrixXd & H, const Eigen::MatrixXd & R,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &)> h,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> z_subtract)
{
  Eigen::VectorXd x_prior = x;

  // 使用自适应测量噪声（如果启用）
  Eigen::MatrixXd effective_R = config_.enable_adaptive_noise ?
                                get_adaptive_measurement_noise() : R;

  // 计算协方差矩阵 S = H*P*H' + R
  Eigen::MatrixXd S = H * P * H.transpose() + effective_R;

  // 使用SVD进行稳定求逆
  svd_stats_.decomposition_count++;
  Eigen::MatrixXd S_inv = stable_matrix_inverse(S);

  // 计算卡尔曼增益
  Eigen::MatrixXd K = P * H.transpose() * S_inv;

  // 稳定的协方差更新
  Eigen::MatrixXd I_KH = I - K * H;
  P = I_KH * P * I_KH.transpose() + K * effective_R * K.transpose();

  // 确保协方差矩阵保持正定
  if (is_well_conditioned(P)) {
    ensure_positive_definite();
  }

  // 状态更新
  x = x_add(x, K * z_subtract(z, h(x)));

  // 更新残差历史以用于自适应噪声估计
  if (config_.enable_adaptive_noise) {
    Eigen::VectorXd residual = z_subtract(z, h(x));
    update_noise_estimates(residual);
  }

  // 卡方检验（复用基础EKF功能）
  residual = z_subtract(z, h(x));
  double nis = residual.transpose() * effective_R.inverse() * residual;
  double nees = (x - x_prior).transpose() * P.inverse() * (x - x_prior);

  // 更新NIS/NEES检验（复用基础实现）
  constexpr double nis_threshold = 0.711;
  constexpr double nees_threshold = 0.711;

  data["nis"] = nis;
  data["nees"] = nees;

  if (nis > nis_threshold) data["nis_fail"] = 1.0;
  if (nees > nees_threshold) data["nees_fail"] = 1.0;

  return x;
}

Eigen::MatrixXd SVDEhancedKalmanFilter::stable_matrix_inverse(const Eigen::MatrixXd& matrix) {
  // 条件数监控
  double condition_number = get_condition_number(matrix);

  // 执行SVD分解
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(matrix, Eigen::ComputeThinU | Eigen::ComputeThinV);

  const auto& singular_values = svd.singularValues();
  double max_singular_value = singular_values(0);

  // 计算最优正则化参数
  double optimal_regularization = estimate_optimal_regularization(singular_values);

  // 截断奇异值并计算伪逆
  Eigen::VectorXd pseudo_inverse_singular_values = truncate_singular_values(singular_values);

  // 构建伪逆矩阵
  Eigen::MatrixXd pseudo_inverse = svd.matrixV() *
                                   pseudo_inverse_singular_values.asDiagonal() *
                                   svd.matrixU().transpose();

  // 更新统计信息
  svd_stats_.avg_condition_number = (svd_stats_.avg_condition_number * (svd_stats_.decomposition_count - 1) +
                                    condition_number) / svd_stats_.decomposition_count;
  svd_stats_.max_condition_number = std::max(svd_stats_.max_condition_number, condition_number);

  if (condition_number > config_.condition_number_threshold) {
    svd_stats_.ill_conditioned_count++;

    // 添加正则化
    Eigen::MatrixXd regularized_inverse = pseudo_inverse +
       Eigen::MatrixXd::Identity(matrix.rows(), matrix.cols()) * optimal_regularization;

    return regularized_inverse;
  }

  return pseudo_inverse;
}

double SVDEhancedKalmanFilter::get_condition_number(const Eigen::MatrixXd& matrix) {
  // 使用JacobiSVD计算条件数
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(matrix);
  const auto& singular_values = svd.singularValues();

  if (singular_values.size() == 0) return 1.0;

  double max_sv = singular_values(0);
  double min_sv = singular_values(singular_values.size() - 1);

  return (min_sv < std::numeric_limits<double>::epsilon()) ?
         std::numeric_limits<double>::max() : max_sv / min_sv;
}

bool SVDEhancedKalmanFilter::is_well_conditioned(const Eigen::MatrixXd& matrix) {
  return get_condition_number(matrix) < config_.condition_number_threshold;
}

Eigen::VectorXd SVDEhancedKalmanFilter::truncate_singular_values(const Eigen::VectorXd& singular_values) {
  Eigen::VectorXd truncated = Eigen::VectorXd::Zero(singular_values.size());

  for (int i = 0; i < singular_values.size(); ++i) {
    double sv = singular_values(i);
    double max_sv = singular_values(0);

    // 基于两个标准进行截断：数值稳定性和能量保留
    bool above_numerical_threshold = sv > config_.min_singular_value_ratio * max_sv;
    bool above_energy_threshold = sv > config_.regularization_factor * max_sv;

    if (above_numerical_threshold && above_energy_threshold) {
      truncated(i) = 1.0 / sv;
    } else {
      truncated(i) = 0.0;
      svd_stats_.truncation_count++;
    }
  }

  return truncated;
}

double SVDEhancedKalmanFilter::estimate_optimal_regularization(const Eigen::VectorXd& singular_values) {
  // 使用L曲线准则或GCV方法估计最优正则化参数
  // 简化的实现：基于奇异值谱的相对大小
  double max_sv = singular_values(0);

  // 寻找拐点（奇异值明显下降的点）
  double curvature_threshold = 0.1;
  int knee_point = singular_values.size() - 1;

  for (int i = 1; i < singular_values.size() - 1; ++i) {
    double curvature = std::abs(singular_values(i-1) - 2*singular_values(i) + singular_values(i+1));
    if (curvature > curvature_threshold * max_sv) {
      knee_point = i;
      break;
    }
  }

  return config_.regularization_factor * singular_values(knee_point);
}

void SVDEhancedKalmanFilter::update_noise_estimates(const Eigen::VectorXd& residual) {
  // 维护残差历史
  residual_history_.push_back(residual);
  if (residual_history_.size() > config_.noise_history_size) {
    residual_history_.pop_front();
  }

  if (residual_history_.size() < config_.noise_history_size / 2) return;

  // 构建残差矩阵进行SVD分析
  int residual_dim = residual.size();
  int history_size = residual_history_.size();

  Eigen::MatrixXd residual_matrix(residual_dim, history_size);
  for (int i = 0; i < history_size; ++i) {
    residual_matrix.col(i) = residual_history_[i];
  }

  // 计算样本协方差
  Eigen::VectorXd mean_residual = residual_matrix.rowwise().mean();
  Eigen::MatrixXd centered = residual_matrix.colwise() - mean_residual;
  Eigen::MatrixXd sample_covariance = (centered * centered.transpose()) / (history_size - 1);

  // 使用SVD进行稳健估计
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(sample_covariance);
  Eigen::VectorXd robust_singular_values = truncate_singular_values(svd.singularValues());

  // 重构噪声协方差矩阵
  adaptive_R_ = svd.matrixU() * robust_singular_values.asDiagonal() * svd.matrixU().transpose();
}

Eigen::MatrixXd SVDEhancedKalmanFilter::get_adaptive_measurement_noise() {
  if (residual_history_.size() < config_.noise_history_size / 2) {
    return Eigen::MatrixXd::Identity(adaptive_R_.rows(), adaptive_R_.cols()) * 0.01; // 默认噪声
  }

  return adaptive_R_;
}

SVDEhancedKalmanFilter::ReducedDimensionResult
SVDEhancedKalmanFilter::reduce_dimension(const Eigen::MatrixXd& matrix) {
  ReducedDimensionResult result;

  Eigen::JacobiSVD<Eigen::MatrixXd> svd(matrix, Eigen::ComputeFullU | Eigen::ComputeFullV);
  const Eigen::VectorXd& singular_values = svd.singularValues();

  // 计算累积能量
  double total_energy = singular_values.array().square().sum();
  double cumulative_energy = 0.0;
  result.effective_rank = 0;

  for (int i = 0; i < singular_values.size(); ++i) {
    cumulative_energy += singular_values(i) * singular_values(i);
    if (cumulative_energy / total_energy > config_.energy_threshold) {
      result.effective_rank = i + 1;
      break;
    }
  }

  // 提取主要奇异向量和奇异值
  result.U_principal = svd.matrixU().leftCols(result.effective_rank);
  result.singular_values = singular_values.head(result.effective_rank);

  return result;
}

void SVDEhancedKalmanFilter::ensure_positive_definite() {
  // 使用SVD确保协方差矩阵的正定性
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(P);

  Eigen::VectorXd corrected_singular_values = svd.singularValues();

  // 确保所有奇异值为正且大于阈值
  double min_sv = config_.regularization_factor * corrected_singular_values(0);

  for (int i = 0; i < corrected_singular_values.size(); ++i) {
    if (corrected_singular_values(i) < min_sv) {
      corrected_singular_values(i) = min_sv;
    }
  }

  // 重构正定协方差矩阵
  P = svd.matrixU() * corrected_singular_values.asDiagonal() * svd.matrixU().transpose();

  // 确保对称性
  P = (P + P.transpose()) / 2.0;
}

} // namespace tools

#endif // TOOLS__SVD_ENHANCED_KALMAN_CPP