// Copyright 2022 Chen Jun

#ifndef ARMOR_PROCESSOR__KALMAN_FILTER_HPP_
#define ARMOR_PROCESSOR__KALMAN_FILTER_HPP_

#include <Eigen/Dense>
#include <functional>
#include <deque>

namespace rm_auto_aim
{
/**
 * @class ExtendedKalmanFilter
 * @brief 扩展卡尔曼滤波器（EKF）类
 *
 * - 支持任意非线性状态转移函数 f(x) 和观测函数 h(x)
 * - 通过外部提供的雅可比函数进行线性化
 * - 支持按需更新过程噪声与测量噪声协方差矩阵
 * - [IMM-Phase A] 新增 get_S() / get_innovation() 接口，供 IMMTracker 计算似然使用
 */
class ExtendedKalmanFilter
{
public:
    ExtendedKalmanFilter() = default;

    using VecVecFunc = std::function<Eigen::VectorXd(const Eigen::VectorXd &)>;
    using VecMatFunc = std::function<Eigen::MatrixXd(const Eigen::VectorXd &)>;
    using VoidMatFunc = std::function<Eigen::MatrixXd()>;

    explicit ExtendedKalmanFilter(
        const VecVecFunc & f, const VecVecFunc & h, const VecMatFunc & j_f, const VecMatFunc & j_h,
        const VoidMatFunc & u_q, const VecMatFunc & u_r, const Eigen::MatrixXd & P0);

    /**
     * @brief 设置滤波器的当前状态向量（用于初始化或强制重设）
     * @param x0 初始状态向量
     */
    void setState(const Eigen::VectorXd & x0);

    /**
     * @brief 执行预测步骤，返回先验状态 x_pri
     * @return 预测得到的先验状态向量
     */
    Eigen::VectorXd predict();

    /**
     * @brief 使用测量 z 执行更新步骤（内部调用 update_R 生成 R）
     * @param z 测量向量
     * @return 更新后的后验状态向量
     */
    Eigen::VectorXd update(const Eigen::VectorXd & z);

    /**
     * @brief 使用测量 z 和外部传入的 adaptive_R 执行更新步骤
     * @param z          测量向量
     * @param adaptive_R 外部指定的测量噪声矩阵（用于前哨战高度突变等特殊场景）
     * @return 更新后的后验状态向量
     */
    Eigen::VectorXd update(const Eigen::VectorXd & z, Eigen::MatrixXd & adaptive_R);

    /** @brief 获取当前测量噪声协方差矩阵 R */
    Eigen::MatrixXd get_R();

    /** @brief 获取后验状态向量 */
    Eigen::VectorXd get_x_post();

    /** @brief 获取后验误差协方差矩阵 P */
    Eigen::MatrixXd get_P() const;

    /** @brief 强制设置后验误差协方差矩阵 P（用于 jump handle / r 裁剪后的同步） */
    void set_P(const Eigen::MatrixXd & P);

    // ----------------------------------------------------------------
    // [IMM-Phase A] 新增接口：供 IMMTracker 计算似然概率使用
    // ----------------------------------------------------------------

    /**
     * @brief 获取最近一次 update() 计算的新息协方差矩阵 S = H*P_pri*H^T + R
     * @note  仅在 update() 调用之后有效；predict() 后调用返回上一帧的值
     * @return 新息协方差矩阵（量测维度 × 量测维度）
     */
    Eigen::MatrixXd get_S() const;

    /**
     * @brief 获取最近一次 update() 计算的新息向量 e = z - h(x_pri)
     * @note  仅在 update() 调用之后有效
     * @return 新息向量（量测维度）
     */
    Eigen::VectorXd get_innovation() const;

    // ----------------------------------------------------------------
    // [IMM-Phase A] 自适应 Q 接口（Phase A 中关闭，接口保留供后续使用）
    // ----------------------------------------------------------------

    /**
     * @brief 启用自适应 Q，设置平滑系数和窗口大小
     * @note  [IMM-Phase A] 此功能在 Phase A 中已停用，IMM 框架下由模型分工替代自适应 Q
     */
    void enableAdaptiveQ(double alpha, int window_size);

    /** @brief 获取当前自适应 Q 的缩放系数 lambda */
    double getAdaptiveLambda() const;

    /**
     * @brief 将 EKF 恢复到初始干净状态
     * @param x0 初始后验状态
     * @param P0 初始后验协方差
     */
    void reset(const Eigen::VectorXd & x0, const Eigen::MatrixXd & P0);


private:
    /** @brief 过程非线性函数 f(x) */
    VecVecFunc f;
    /** @brief 观测非线性函数 h(x) */
    VecVecFunc h;
    /** @brief f 的雅可比矩阵计算函数 */
    VecMatFunc jacobian_f;
    Eigen::MatrixXd F;
    /** @brief h 的雅可比矩阵计算函数 */
    VecMatFunc jacobian_h;
    Eigen::MatrixXd H;
    /** @brief 过程噪声协方差更新函数（无参） */
    VoidMatFunc update_Q;
    Eigen::MatrixXd Q;
    /** @brief 测量噪声协方差更新函数 */
    VecMatFunc update_R;
    Eigen::MatrixXd R;

    /** @brief 先验误差协方差 */
    Eigen::MatrixXd P_pri;
    /** @brief 后验误差协方差 */
    Eigen::MatrixXd P_post;

    /** @brief 卡尔曼增益 */
    Eigen::MatrixXd K;

    /** @brief 观测残差（新息）e = z - h(x_pri) */
    Eigen::VectorXd e;

    // [IMM-Phase A] 新息协方差 S = H*P_pri*H^T + R
    // 原来是 update() 内部的局部变量，现在提升为成员变量，供 get_S() 对外暴露
    Eigen::MatrixXd S;

    /** @brief 系统维度：状态维度 n，观测维度 k */
    int n;
    int k;

    /** @brief n 阶单位矩阵（Joseph 稳定化公式用） */
    Eigen::MatrixXd I;

    /** @brief 自适应 Q 的缩放系数（Phase A 中不使用） */
    double adaptive_q_lambda_ = 1.0;
    /** @brief 是否启用自适应 Q（Phase A 中强制为 false） */
    bool use_adaptive_q_ = false;
    double alpha_q_ = 0.35;

    /** @brief NIS 残差滑动窗口（自适应 Q 用） */
    std::deque<double> nis_window_;
    int nis_window_size_ = 10;

    /** @brief 先验状态向量 */
    Eigen::VectorXd x_pri;
    /** @brief 后验状态向量 */
    Eigen::VectorXd x_post;
};

}  // namespace rm_auto_aim

#endif  // ARMOR_PROCESSOR__KALMAN_FILTER_HPP_
