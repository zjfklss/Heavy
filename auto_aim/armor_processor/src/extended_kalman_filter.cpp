// Copyright 2022 Chen Jun

#include "armor_processor/extended_kalman_filter.hpp"
#include "rclcpp/rclcpp.hpp"
#include <memory>
#include <vector>
#include <fstream>
#include <iostream>

namespace rm_auto_aim
{

// ============================================================
// 构造函数
// 参数说明：
//   f    : 状态转移非线性函数 x_{k+1} = f(x_k)
//   h    : 观测非线性函数 z_k = h(x_k)
//   j_f  : f 的雅可比矩阵函数
//   j_h  : h 的雅可比矩阵函数
//   u_q  : 过程噪声协方差更新函数（每帧调用，可依赖 dt）
//   u_r  : 观测噪声协方差更新函数（依赖当前量测）
//   P0   : 初始后验协方差矩阵
// ============================================================
ExtendedKalmanFilter::ExtendedKalmanFilter(
    const VecVecFunc & f, const VecVecFunc & h, const VecMatFunc & j_f, const VecMatFunc & j_h,
    const VoidMatFunc & u_q, const VecMatFunc & u_r, const Eigen::MatrixXd & P0)
: f(f),
  h(h),
  jacobian_f(j_f),
  jacobian_h(j_h),
  update_Q(u_q),
  update_R(u_r),
  P_post(P0),
  n(P0.rows()),
  I(Eigen::MatrixXd::Identity(n, n)),
  x_pri(n),
  x_post(n)
{
    // S 和 e 初始化为零，避免首帧 get_S()/get_innovation() 返回未初始化值
    k = 0;  // 观测维度在首次 update() 中由 H 的行数确定
    e = Eigen::VectorXd::Zero(1);   // 占位，首次 update 后自动替换
    S = Eigen::MatrixXd::Identity(1, 1);  // 占位
}

// ============================================================
// setState：强制设置后验状态（初始化 / jump handle / r 裁剪同步）
// ============================================================
void ExtendedKalmanFilter::setState(const Eigen::VectorXd & x0)
{
    x_post = x0;
}

// ============================================================
// predict：预测步骤
//   1. 计算状态转移雅可比 F = ∂f/∂x|_{x_post}
//   2. 更新过程噪声 Q（可依赖 dt，在 u_q lambda 中捕获）
//   3. 先验状态预测：x_pri = f(x_post)
//   4. 先验协方差传播：P_pri = F * P_post * F^T + Q
//   [IMM-Phase A] 自适应 Q 功能已停用（use_adaptive_q_ 始终为 false）
// ============================================================
Eigen::VectorXd ExtendedKalmanFilter::predict()
{
    F = jacobian_f(x_post);
    Q = update_Q();

    // [IMM-Phase A] 自适应 Q 已停用，此分支不会进入
    // IMM 框架下，模型分工（三套不同的 Q）替代了单模型自适应 Q 的作用
    if (use_adaptive_q_) {
        Q *= adaptive_q_lambda_;
    }

    x_pri = f(x_post);
    P_pri = F * P_post * F.transpose() + Q;

    // 若此帧没有量测更新，先验值作为后验值（TEMP_LOST 时使用）
    x_post = x_pri;
    P_post = P_pri;

    return x_pri;
}

// ============================================================
// update（无外部 R）：内部调用 update_R 生成 R，再调用带 R 版本
// ============================================================
Eigen::VectorXd ExtendedKalmanFilter::update(const Eigen::VectorXd & z)
{
    R = update_R(z);
    return update(z, this->R);
}

// ============================================================
// update（带外部 adaptive_R）：EKF 更新步骤
//   1. 计算观测雅可比 H = ∂h/∂x|_{x_pri}
//   2. 计算新息 e = z - h(x_pri)
//   3. 计算新息协方差 S = H*P_pri*H^T + R
//      [IMM-Phase A] S 由局部变量提升为成员变量，供 IMMTracker::computeLikelihoods() 使用
//   4. 计算卡尔曼增益 K = P_pri * H^T * S^{-1}
//   5. 后验状态更新：x_post = x_pri + K * e
//   6. 后验协方差更新（Joseph 稳定化形式，保证对称正定）：
//      P_post = (I - K*H)*P_pri*(I - K*H)^T + K*R*K^T
// ============================================================
Eigen::VectorXd ExtendedKalmanFilter::update(const Eigen::VectorXd & z, Eigen::MatrixXd & adaptive_R)
{
    H = jacobian_h(x_pri);

    // 新息：观测残差
    e = z - h(x_pri);

    // [IMM-Phase A] S 由局部变量提升为成员变量
    // IMMTracker 在所有子 EKF 完成 update() 后，通过 get_S() 和 get_innovation()
    // 取出 S 和 e，用于计算高斯似然概率（论文公式24）
    S = H * P_pri * H.transpose() + adaptive_R;

    // 卡尔曼增益
    K = P_pri * H.transpose() * S.inverse();

    // 后验状态
    x_post = x_pri + K * e;

    // Joseph 稳定化形式的协方差更新
    // 即使 K 有数值误差，P_post 依然保持对称正定
    Eigen::MatrixXd IKH = I - K * H;
    P_post = IKH * P_pri * IKH.transpose() + K * adaptive_R * K.transpose();

    // [IMM-Phase A] 自适应 Q 已停用，以下分支不会进入
    if (use_adaptive_q_) {
        double nis = e.transpose() * S.inverse() * e;
        double nis_normalized = nis / e.size();
        nis_window_.push_back(nis_normalized);
        if ((int)nis_window_.size() > nis_window_size_) {
            nis_window_.pop_front();
        }
        double nis_mean = 0.0;
        for (auto v : nis_window_) nis_mean += v;
        nis_mean /= nis_window_.size();

        double alpha_up   = alpha_q_;
        double alpha_down = alpha_q_ * 0.5;
        double alpha_use  = (nis_mean > adaptive_q_lambda_) ? alpha_up : alpha_down;
        adaptive_q_lambda_ = (1.0 - alpha_use) * adaptive_q_lambda_ + alpha_use * nis_mean;
        adaptive_q_lambda_ = std::clamp(adaptive_q_lambda_, 0.2, 20.0);
    }

    return x_post;
}

// ============================================================
// Getter 实现
// ============================================================

Eigen::MatrixXd ExtendedKalmanFilter::get_R()
{
    return this->R;
}

Eigen::VectorXd ExtendedKalmanFilter::get_x_post()
{
    return this->x_post;
}

Eigen::MatrixXd ExtendedKalmanFilter::get_P() const
{
    return P_post;
}

void ExtendedKalmanFilter::set_P(const Eigen::MatrixXd & P)
{
    P_post = P;
}

// ============================================================
// [IMM-Phase A] 新增 getter：对外暴露新息协方差 S 和新息 e
// ============================================================

/**
 * @brief 返回最近一次 update() 计算的新息协方差矩阵 S = H*P_pri*H^T + R
 *
 * IMMTracker::computeLikelihoods() 中使用此值计算高斯似然（论文公式24）：
 *   Λ^m = 1/sqrt((2π)^d * |S^m|) * exp(-0.5 * e^T * (S^m)^{-1} * e)
 */
Eigen::MatrixXd ExtendedKalmanFilter::get_S() const
{
    return S;
}

/**
 * @brief 返回最近一次 update() 计算的新息向量 e = z - h(x_pri)
 *
 * 同样供 IMMTracker::computeLikelihoods() 使用
 */
Eigen::VectorXd ExtendedKalmanFilter::get_innovation() const
{
    return e;
}

// ============================================================
// 自适应 Q 相关（Phase A 中已停用，接口保留）
// ============================================================
void ExtendedKalmanFilter::enableAdaptiveQ(double alpha, int window_size)
{
    // [IMM-Phase A] 此函数在 Phase A 中不应被调用
    // IMM 框架下，三个子模型的不同 Q 矩阵替代了单模型自适应 Q 的职责
    // 接口保留供 Phase C 评估是否重新引入弱自适应
    use_adaptive_q_ = true;
    alpha_q_ = alpha;
    nis_window_size_ = window_size;
    adaptive_q_lambda_ = 1.0;
    nis_window_.clear();
}

double ExtendedKalmanFilter::getAdaptiveLambda() const
{
    return adaptive_q_lambda_;
}

void ExtendedKalmanFilter::reset(const Eigen::VectorXd & x0, const Eigen::MatrixXd & P0)
{
    x_post = x0;
    x_pri  = x0;

    P_post = P0;
    P_pri  = P0;

    // 这些量在首次 predict/update 前没有实际物理意义，清零/置单位即可
    F = Eigen::MatrixXd::Identity(n, n);
    H = Eigen::MatrixXd::Zero(1, n);

    Q = Eigen::MatrixXd::Zero(n, n);
    R = Eigen::MatrixXd::Identity(1, 1);

    K = Eigen::MatrixXd::Zero(n, 1);

    e = Eigen::VectorXd::Zero(1);
    S = Eigen::MatrixXd::Identity(1, 1);

    k = 0;

    // 自适应 Q 相关缓存也清掉
    adaptive_q_lambda_ = 1.0;
    nis_window_.clear();
}


}  // namespace rm_auto_aim
