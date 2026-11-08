// imm_tracker.cpp
// [IMM-Phase A] 交互式多模型跟踪器实现
// 对应论文公式编号已在每个关键步骤注明

#include "armor_processor/imm_tracker.hpp"
#include <cmath>
#include <numeric>
#include <stdexcept>
#include "rclcpp/rclcpp.hpp"

namespace rm_auto_aim
{

// ============================================================
// 构造函数
// ============================================================
IMMTracker::IMMTracker(
    const VecVecFunc & f,
    const VecVecFunc & h,
    const VecMatFunc & j_f,
    const VecMatFunc & j_h,
    const std::array<VoidMatFunc, N> & u_q_list,
    const VecMatFunc & u_r,
    const Eigen::MatrixXd & P0)
{
    for (int m = 0; m < N; ++m) {
        models_[m] = EKF(f, h, j_f, j_h, u_q_list[m], u_r, P0);
    }

    mu_.setConstant(1.0 / N);
    mu_prev_ = mu_;

    M_init_ << 0.80, 0.10, 0.10,
          0.10, 0.80, 0.10,
          0.10, 0.10, 0.80;
    M_ = M_init_;

    P_init_ = P0;
    int state_dim = P0.rows();
    for (int m = 0; m < N; ++m) {
        x_mixed_[m] = Eigen::VectorXd::Zero(state_dim);
        P_mixed_[m] = P0;
    }
    x_fused_ = Eigen::VectorXd::Zero(state_dim);
    P_fused_ = P0;
    c_j_.setZero();
    likelihood_.setZero();
}

// ============================================================
// setState：同步更新所有子模型状态
// ============================================================
void IMMTracker::setState(const Eigen::VectorXd & x0)
{
    for (int m = 0; m < N; ++m) {
        models_[m].setState(x0);
    }
    x_fused_ = x0;
}

// ============================================================
// set_P：同步设置所有子模型协方差
// ============================================================
void IMMTracker::set_P(const Eigen::MatrixXd & P)
{
    for (int m = 0; m < N; ++m) {
        models_[m].set_P(P);
    }
    P_fused_ = P;
}

// ============================================================
// predict：仅预测步骤（无量测，TEMP_LOST 时使用）
// ============================================================
Eigen::VectorXd IMMTracker::predict()
{
    mixing();

    for (int m = 0; m < N; ++m) {
        models_[m].setState(x_mixed_[m]);
        models_[m].set_P(P_mixed_[m]);
        models_[m].predict();
    }

    fuseOutput();

    return x_fused_;
}

// ============================================================
// peekPrediction：只读先验预测（用于匹配）
// ============================================================
Eigen::VectorXd IMMTracker::peekPrediction()
{
    // 1. 备份当前所有子模型以及融合后的后验状态
    std::array<Eigen::VectorXd, N> backup_x;
    std::array<Eigen::MatrixXd, N> backup_P;
    for (int m = 0; m < N; ++m) {
        backup_x[m] = models_[m].get_x_post();
        backup_P[m] = models_[m].get_P();
    }
    Eigen::VectorXd backup_x_fused = x_fused_;
    Eigen::MatrixXd backup_P_fused = P_fused_;

    // 2. 执行一次完整的 IMM 预测操作，获取本帧真实先验
    Eigen::VectorXd prior_state = predict();

    // 3. 恢复内部状态（彻底撤销 predict 带来的状态前滚副作用）
    for (int m = 0; m < N; ++m) {
        models_[m].setState(backup_x[m]);
        models_[m].set_P(backup_P[m]);
    }
    x_fused_ = backup_x_fused;
    P_fused_ = backup_P_fused;

    return prior_state;
}


// ============================================================
// update：完整 IMM 预测+更新步骤（有量测帧时使用）
// ============================================================
Eigen::VectorXd IMMTracker::update(const Eigen::VectorXd & z)
{
    mixing();

    for (int m = 0; m < N; ++m) {
        // models_[m].setState(x_mixed_[m]);
        // models_[m].set_P(P_mixed_[m]);
        // models_[m].predict();
        models_[m].update(z);
    }

    computeLikelihoods();
    updateWeights();
    updateMarkov();
    fuseOutput();

    return x_fused_;
}

// ============================================================
// Step1: 数据交互（Mixing）
// ============================================================
void IMMTracker::mixing()
{
    int state_dim = x_fused_.size();

    for (int m = 0; m < N; ++m) {
        c_j_(m) = 0.0;
        for (int n = 0; n < N; ++n) {
            c_j_(m) += M_(n, m) * mu_(n);
        }
        if (c_j_(m) < 1e-10) {
            c_j_(m) = 1e-10;
        }
    }

    for (int m = 0; m < N; ++m) {
        x_mixed_[m] = Eigen::VectorXd::Zero(state_dim);

        for (int n = 0; n < N; ++n) {
            double U_nm = M_(n, m) * mu_(n) / c_j_(m);
            x_mixed_[m] += U_nm * models_[n].get_x_post();
        }

        P_mixed_[m] = Eigen::MatrixXd::Zero(state_dim, state_dim);
        for (int n = 0; n < N; ++n) {
            double U_nm = M_(n, m) * mu_(n) / c_j_(m);
            Eigen::VectorXd dx = models_[n].get_x_post() - x_mixed_[m];
            P_mixed_[m] += U_nm * (models_[n].get_P() + dx * dx.transpose());
        }
    }
}

// ============================================================
// Step3: 似然计算（对数稳定版）
// [修复] 维度不要写死为 5
// ============================================================
void IMMTracker::computeLikelihoods()
{
    const double log2pi = std::log(2.0 * M_PI);

    Eigen::Matrix<double, N, 1> log_likelihood;

    for (int m = 0; m < N; ++m) {
        Eigen::MatrixXd S_m = models_[m].get_S();
        Eigen::VectorXd e_m = models_[m].get_innovation();

        // [修复] 维度从 e_m 自动获取
        const int d = e_m.size();

        Eigen::LLT<Eigen::MatrixXd> llt(S_m);
        if (llt.info() != Eigen::Success) {
            RCLCPP_WARN(rclcpp::get_logger("imm_tracker"),
                "Model %d: S matrix is not positive definite, using fallback likelihood.", m);
            log_likelihood(m) = -1e6;
            continue;
        }

        double log_det_S = 2.0 * llt.matrixL().toDenseMatrix().diagonal().array().log().sum();

        // [修复] 马氏距离用 llt.solve() 而不是 inverse()
        double mahal_sq = e_m.transpose() * llt.solve(e_m);

        log_likelihood(m) = -0.5 * (d * log2pi + log_det_S + mahal_sq);
    }

    double log_max = log_likelihood.maxCoeff();
    for (int m = 0; m < N; ++m) {
        likelihood_(m) = std::exp(log_likelihood(m) - log_max);
    }
}

// ============================================================
// Step4: 权重更新（贝叶斯更新）
// ============================================================
void IMMTracker::updateWeights()
{
    mu_prev_ = mu_;

    double zeta_total = 0.0;
    for (int m = 0; m < N; ++m) {
        zeta_total += likelihood_(m) * c_j_(m);
    }

    if (zeta_total < 1e-10) {
        RCLCPP_WARN(rclcpp::get_logger("imm_tracker"),
            "Total likelihood near zero, keeping previous weights.");
        return;
    }

    for (int m = 0; m < N; ++m) {
        mu_(m) = likelihood_(m) * c_j_(m) / zeta_total;
    }

    double sum = mu_.sum();
    if (sum > 1e-10) {
        mu_ /= sum;
    }
}

// ============================================================
// Step5: 自适应更新马尔可夫转移矩阵
// [修复] 对角增强 + 非对角重分配，避免整列缩放被归一化抵消
// ============================================================
void IMMTracker::updateMarkov()
{
    const double alpha = 3.0;
    const double diag_min = 0.60;
    const double diag_max = 0.95;

    Eigen::Matrix<double, N, N> M_new = M_;

    for (int m = 0; m < N; ++m) {
        double delta_mu = mu_(m) - mu_prev_(m);
        double gain = std::exp(alpha * delta_mu);
        gain = std::clamp(gain, 0.5, 2.0);

        double old_diag = M_(m, m);
        double new_diag = old_diag * gain;
        new_diag = std::clamp(new_diag, diag_min, diag_max);

        M_new(m, m) = new_diag;

        double remain = 1.0 - new_diag;

        double offdiag_sum = 0.0;
        for (int n = 0; n < N; ++n) {
            if (n != m) {
                offdiag_sum += M_(n, m);
            }
        }

        if (offdiag_sum > 1e-12) {
            for (int n = 0; n < N; ++n) {
                if (n != m) {
                    M_new(n, m) = remain * M_(n, m) / offdiag_sum;
                }
            }
        } else {
            for (int n = 0; n < N; ++n) {
                if (n != m) {
                    M_new(n, m) = remain / (N - 1);
                }
            }
        }
    }

    for (int m = 0; m < N; ++m) {
        double col_sum = M_new.col(m).sum();
        if (col_sum > 1e-12) {
            M_new.col(m) /= col_sum;
        } else {
            M_new.col(m).setConstant(1.0 / N);
        }
    }

    M_ = M_new;
}

// ============================================================
// Step6: 加权融合输出
// ============================================================
void IMMTracker::fuseOutput()
{
    int state_dim = x_fused_.size();

    x_fused_ = Eigen::VectorXd::Zero(state_dim);
    for (int m = 0; m < N; ++m) {
        x_fused_ += mu_(m) * models_[m].get_x_post();
    }

    P_fused_ = Eigen::MatrixXd::Zero(state_dim, state_dim);
    for (int m = 0; m < N; ++m) {
        Eigen::VectorXd dx = models_[m].get_x_post() - x_fused_;
        P_fused_ += mu_(m) * (models_[m].get_P() + dx * dx.transpose());
    }
}

// ============================================================
// Getter 实现
// ============================================================

Eigen::MatrixXd IMMTracker::get_P() const
{
    return P_fused_;
}

Eigen::VectorXd IMMTracker::get_x_post()
{
    return x_fused_;
}

Eigen::MatrixXd IMMTracker::get_R()
{
    return models_[0].get_R();
}

std::array<double, IMMTracker::N> IMMTracker::getModelWeights() const
{
    std::array<double, N> weights;
    for (int m = 0; m < N; ++m) {
        weights[m] = mu_(m);
    }
    return weights;
}

Eigen::Matrix<double, IMMTracker::N, IMMTracker::N> IMMTracker::getMarkovMatrix() const
{
    return M_;
}

void IMMTracker::reset()
{
    const int state_dim = P_init_.rows();

    // 恢复模型权重
    mu_.setConstant(1.0 / N);
    mu_prev_ = mu_;

    // 恢复马尔可夫矩阵
    M_ = M_init_;

    // 清空中间量
    c_j_.setZero();
    likelihood_.setZero();

    // 默认零状态
    Eigen::VectorXd x0 = Eigen::VectorXd::Zero(state_dim);

    for (int m = 0; m < N; ++m) {
        models_[m].reset(x0, P_init_);
        x_mixed_[m] = x0;
        P_mixed_[m] = P_init_;
    }

    x_fused_ = x0;
    P_fused_ = P_init_;
}

void IMMTracker::reset(const Eigen::VectorXd & x0, const Eigen::MatrixXd & P0)
{
    mu_.setConstant(1.0 / N);
    mu_prev_ = mu_;

    M_ = M_init_;

    c_j_.setZero();
    likelihood_.setZero();

    for (int m = 0; m < N; ++m) {
        models_[m].reset(x0, P0);
        x_mixed_[m] = x0;
        P_mixed_[m] = P0;
    }

    x_fused_ = x0;
    P_fused_ = P0;
}


}  // namespace rm_auto_aim
