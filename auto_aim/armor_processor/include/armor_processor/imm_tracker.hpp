// imm_tracker.hpp
// [IMM-Phase A] 交互式多模型跟踪器（IMM-EKF）
//
// 实现论文《A Maneuvering Extended Target Tracking IMM Algorithm
// Based on Second-Order EKF》中的 IMM 框架部分（公式3~38）。
//
// 注意：论文中针对雷达扩展目标的形状参数估计（椭圆参数、伪量测）
// 在 RM 自瞄场景下不需要，已裁剪。"形状量" r 由 EKF 状态向量第8维承担。
//
// 三个子模型（统一11维状态，只有 Q 不同）：
//   Model 0: SteadyMotion      — 平稳移动，小 Q，适合匀速横移/前哨战
//   Model 1: TranslationalManeuver — 平移机动，中等 Q，即原单 EKF 参数
//   Model 2: SpinManeuver      — 自旋机动，yaw 方向大 Q，适合小陀螺
//
// 马尔可夫矩阵约定：M(n|m) 表示"目标模型 m 的输入来自源模型 n 的概率"
// 即按列归一化：sum_n M(n|m) = 1

#ifndef ARMOR_PROCESSOR__IMM_TRACKER_HPP_
#define ARMOR_PROCESSOR__IMM_TRACKER_HPP_

#include <Eigen/Dense>
#include <array>
#include "armor_processor/extended_kalman_filter.hpp"

namespace rm_auto_aim
{

class IMMTracker
{
public:
    // 子模型数量（固定为3）
    static constexpr int N = 3;

    using EKF = ExtendedKalmanFilter;
    using VecVecFunc = EKF::VecVecFunc;
    using VecMatFunc = EKF::VecMatFunc;
    using VoidMatFunc = EKF::VoidMatFunc;

    /**
     * @brief 构造 IMM 跟踪器
     *
     * @param f        状态转移函数（三个模型共用）
     * @param h        观测函数（三个模型共用）
     * @param j_f      f 的雅可比函数（三个模型共用）
     * @param j_h      h 的雅可比函数（三个模型共用）
     * @param u_q_list 三个模型各自的过程噪声更新函数
     *                 [0]=SteadyMotion, [1]=TranslationalManeuver, [2]=SpinManeuver
     * @param u_r      观测噪声更新函数（三个模型共用）
     * @param P0       初始后验协方差矩阵
     */
    IMMTracker(
        const VecVecFunc & f,
        const VecVecFunc & h,
        const VecMatFunc & j_f,
        const VecMatFunc & j_h,
        const std::array<VoidMatFunc, N> & u_q_list,
        const VecMatFunc & u_r,
        const Eigen::MatrixXd & P0);

    /**
     * @brief 初始化/重置所有子模型的状态向量
     * @param x0 初始状态（来自 initEKF 或 handleArmorJump）
     * @note  必须同时更新所有子模型，否则下一帧 mixing 时旧状态会污染混合初态
     */
    void setState(const Eigen::VectorXd & x0);

    /**
     * @brief 强制设置所有子模型的后验协方差矩阵
     * @param P 协方差矩阵（来自 jump handle / r 裁剪后的同步）
     */
    void set_P(const Eigen::MatrixXd & P);

    /**
     * @brief 执行 IMM 预测步骤（无量测帧，TEMP_LOST 时使用）
     *
     * 流程：mixing → 各模型 predict → 融合输出
     * @return 融合后的先验状态向量
     */
    Eigen::VectorXd predict();

    /**
     * @brief 获取当前帧的预测先验状态（仅供数据关联匹配使用）
     * @note  此函数内部会模拟一次预测，但不会修改滤波器的内部实际后验状态，
     *        避免与 update(z) 产生 double prediction。
     */
    Eigen::VectorXd peekPrediction();


    /**
     * @brief 执行完整的 IMM 预测+更新步骤（有量测帧时使用）
     *
     * 流程：mixing → 各模型 predict+update → 计算似然 → 更新权重
     *       → 自适应更新马尔可夫矩阵 → 融合输出
     * 对应论文图2的完整 IMM 循环（公式3~38）
     *
     * @param z 当前量测向量（5维）
     * @return  融合后的后验状态向量
     */
    Eigen::VectorXd update(const Eigen::VectorXd & z);

    /**
     * @brief 获取融合后的后验协方差矩阵
     * @return P_fused（融合协方差，公式28）
     */
    Eigen::MatrixXd get_P() const;

    /**
     * @brief 获取融合后的后验状态向量（等价于最近一次 update/predict 的返回值）
     */
    Eigen::VectorXd get_x_post();

    /**
     * @brief 获取第0个子模型的 R 矩阵（用于兼容旧接口）
     * @note  三个模型共用同一个 u_r，取任一模型的 R 均等价
     */
    Eigen::MatrixXd get_R();

    /**
     * @brief 获取当前三个模型的权重（用于 PlotJuggler 调试）
     * @return 长度为 N 的数组，mu_[0..2]，总和为1
     */
    std::array<double, N> getModelWeights() const;

    /**
     * @brief 获取当前马尔可夫转移矩阵（用于调试）
     * @return 3×3 矩阵，按列归一化
     */
    Eigen::Matrix<double, N, N> getMarkovMatrix() const;

    /**
     * @brief 恢复 IMM 到构造后的干净初始状态
     * @note  不指定目标状态时，所有子模型状态清零，协方差恢复为构造时的 P0
     */
    void reset();

    /**
     * @brief 恢复 IMM 到干净状态，并设置新的目标初值
     * @param x0 新目标的初始状态
     * @param P0 新目标的初始协方差
     */
    void reset(const Eigen::VectorXd & x0, const Eigen::MatrixXd & P0);


private:
    // ----------------------------------------------------------------
    // IMM 六步流程的内部实现（对应论文 Section III-B）
    // ----------------------------------------------------------------

    /**
     * @brief Step1: 数据交互（Mixing）
     *
     * 对每个目标模型 m，计算混合初始状态和协方差：
     *   ζ_m = Σ_n M(n|m) * μ(n)                         [论文公式8]
     *   U(n|m) = M(n|m) * μ(n) / ζ_m                    [论文公式7]
     *   x_mixed[m] = Σ_n x[n] * U(n|m)                  [论文公式3]
     *   P_mixed[m] = Σ_n {P[n] + Δx[n]*Δx[n]^T} * U(n|m) [论文公式5]
     *
     * 结果存入 x_mixed_[] / P_mixed_[] / c_j_[]
     */
    void mixing();

    /**
     * @brief Step3: 似然计算（对数稳定版）
     *
     * 对每个模型 m，计算高斯似然（论文公式24）：
     *   Λ^m = N(e^m; 0, S^m)
     *       = 1/√((2π)^d |S^m|) * exp(-0.5 * (e^m)^T * (S^m)^{-1} * e^m)
     *
     * 使用 log-sum-exp 技巧避免数值下溢：
     *   log_Λ[m] = -0.5*(d*log(2π) + log|S^m| + (e^m)^T*(S^m)^{-1}*e^m)
     *   Λ[m] = exp(log_Λ[m] - log_Λ_max)
     *
     * e^m 和 S^m 分别通过 models_[m].get_innovation() 和 models_[m].get_S() 获取
     *
     * @param z 当前量测（仅用于维度，实际新息已在 EKF update 中计算）
     */
    void computeLikelihoods();

    /**
     * @brief Step4: 权重更新（贝叶斯更新）
     *
     *   ζ_total = Σ_m Λ^m * ζ_m（其中 ζ_m = c_j_[m]，来自Step1）
     *   μ_new(m) = Λ^m * ζ_m / ζ_total              [论文公式25-26]
     *
     * 更新前先保存 mu_prev_（供Step5自适应马尔可夫矩阵使用）
     */
    void updateWeights();

    /**
     * @brief Step5: 自适应更新马尔可夫转移矩阵
     *
     *   θ^m = exp(μ_new(m) - μ_prev(m))              [论文公式35]
     *   M̂(n|m) = θ^m * M(n|m)                        [论文公式36]
     *   M_new(n|m) = M̂(n|m) / Σ_n M̂(n|m)            [论文公式38，按列归一化]
     *
     * θ^m 按列缩放：第 m 列所有元素同乘 θ^m，再归一化
     * 作用：权重上升的模型吸引更多转入概率，加速模型切换响应
     */
    void updateMarkov();

    /**
     * @brief Step6: 加权融合输出
     *
     *   X_fused = Σ_m x[m] * μ_new(m)                [论文公式27]
     *   P_fused = Σ_m μ_new(m)*{P[m] + Δx[m]*Δx[m]^T} [论文公式28]
     *
     * 融合结果存入 x_fused_ 和 P_fused_
     */
    void fuseOutput();

    // ----------------------------------------------------------------
    // 成员变量
    // ----------------------------------------------------------------

    /** @brief 三个子 EKF 实例（各自独立维护状态和协方差） */
    std::array<EKF, N> models_;

    /** @brief 当前模型权重 μ[0..2]，总和始终为1（论文 μ_k）*/
    Eigen::Matrix<double, N, 1> mu_;

    /** @brief 上一时刻模型权重（用于自适应马尔可夫矩阵更新，论文 μ_{k-1}）*/
    Eigen::Matrix<double, N, 1> mu_prev_;

    /**
     * @brief 马尔可夫转移矩阵 M（3×3，按列归一化）
     *
     * M(n, m) 表示"目标模型 m 的混合输入中，来自源模型 n 的概率"
     * 每列之和 = 1
     * 初始化为对角线高（如0.80）的矩阵
     */
    Eigen::Matrix<double, N, N> M_;

    // --- 中间变量（每帧计算，无需跨帧持久化）---

    /** @brief 每个目标模型的混合初始状态（Step1输出，Step2输入）*/
    std::array<Eigen::VectorXd, N> x_mixed_;

    /** @brief 每个目标模型的混合初始协方差（Step1输出，Step2输入）*/
    std::array<Eigen::MatrixXd, N> P_mixed_;

    /** @brief 每个目标模型的归一化因子 ζ_m（Step1计算，Step4复用）*/
    Eigen::Matrix<double, N, 1> c_j_;

    /** @brief 每个模型的似然概率 Λ^m（Step3计算，Step4使用）*/
    Eigen::Matrix<double, N, 1> likelihood_;

    /** @brief 融合后的后验状态向量（Step6输出，对外暴露）*/
    Eigen::VectorXd x_fused_;

    /** @brief 融合后的后验协方差矩阵（Step6输出，对外暴露）*/
    Eigen::MatrixXd P_fused_;

    /** @brief 构造时的初始马尔可夫矩阵 */
    Eigen::Matrix<double, N, N> M_init_;

    /** @brief 构造时的初始后验协方差 */
    Eigen::MatrixXd P_init_;

};

}  // namespace rm_auto_aim

#endif  // ARMOR_PROCESSOR__IMM_TRACKER_HPP_
