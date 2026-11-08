// Copyright 2022 Chen Jun
// Licensed under the MIT License.

/**
 * @file pnp_solver.hpp
 * @brief 为检测到的装甲求解 PnP 并计算与图像中心的距离辅助方法。
 *
 * `PnPSolver` 利用 OpenCV 的 `solvePnP` 将装甲顶点的 2D 图像坐标转换为
 * 相机坐标系下的 3D 位姿。类内部保有小/大装甲的标准 3D 模型点，并提供
 * 计算像素点到相机主点距离的辅助函数。
 */

 #ifndef ARMOR_DETECTOR__PNP_SOLVER_HPP_
 #define ARMOR_DETECTOR__PNP_SOLVER_HPP_
 
 #include <geometry_msgs/msg/point.hpp>
 #include <opencv2/core.hpp>
 
 // STD
 #include <array>
 #include <map>
 #include <string>
 #include <vector>
 
 #include "armor_detector/armor.hpp"
 #include <rclcpp/rclcpp.hpp>
 
 namespace rm_auto_aim
 {
 class PnPSolver
 {
 public:
   /**
    * @brief 根据相机内参和畸变系数，构建一个PnP解算器。
    * @param camera_matrix 以行为主序的3x3内参矩阵（9元素数组）
    * @param distortion_coefficients 畸变系数向量 (k1,k2,p1,p2,...)
    */
   PnPSolver(
     const std::array<double, 9> & camera_matrix,
     const std::vector<double> & distortion_coefficients);
 
    /**
     * @brief 对检测到的装甲板进行一个 Perspective-n-Point 运算
     * @param armor Input armor containing 4 image points (from lights)
     * @param rvec Output rotation vector (Rodrigues)
     * @param tvec Output translation vector
     * @return true if PnP succeeded
     */
    bool solvePnP(const Armor & armor, cv::Mat & rvec, cv::Mat & tvec);

    /**
     * @brief PnP 解算（IPPE 双解消歧版，抑制 yaw 翻转抖动）
     *
     * 平面 4 点 PnP 存在两个重投影误差相近的解（yaw 镜像翻转），
     * 仅返回单解的迭代法会在两个分支间跳变，表现为偏航角突跳。
     * 本函数流程：
     * 1. SOLVEPNP_IPPE 一次性求出全部候选解（共面情况下最多 2 个）及其重投影误差；
     * 2. 消歧：
     *    - 若该编号装甲板有上一帧解 -> 选与上一帧旋转测地距离最小的解（时序一致性）；
     *      但若该解重投影误差明显大于误差最小解（疑似跟错分支），回退到误差最小解；
     *    - 否则 -> 选重投影误差最小的解（兜底）；
     * 3. 以选中解为初值做一次 SOLVEPNP_ITERATIVE 抛光（不会跳分支）；
     * 4. 历史按 armor.number 分别缓存，避免同帧多块装甲板互相污染。
     *
     * 注意：小陀螺模式下装甲板真实角度/位置本身会跳变，时序消歧只在
     * 旋转连续时生效（受 max_rot_jump_ 门限约束），不会强行抹平真实跳变。
     *
     * @param armor Input armor containing 4 image points
     * @param rvec Output rotation vector (Rodrigues)
     * @param tvec Output translation vector
     * @return true if PnP succeeded
     */
    bool solvePnPPCABA(const Armor & armor, cv::Mat & rvec, cv::Mat & tvec);

    /**
     * @brief Compute pixel distance between an image point and camera center.
     * @param image_point Image coordinate (2D)
     * @return float Euclidean pixel distance to principal point
     */
    float calculateDistanceToCenter(const cv::Point2f & image_point);
    

    float calcReprojectionError(const Armor & armor, const cv::Mat & rvec, const cv::Mat & tvec);

  private:
    /**
     * @brief 两个旋转之间的测地距离（弧度）
     *
     * 计算 R1、R2 之差对应的旋转角 = acos((trace(R1^T R2) - 1) / 2)，
     * 用于时序消歧时衡量候选解与上一帧解的旋转接近程度。
     */
    double rotationGeodesicDistance(const cv::Mat & R1, const cv::Mat & R2);

   cv::Mat camera_matrix_;   ///< Camera intrinsic matrix (3x3, CV_64F)
   cv::Mat dist_coeffs_;     ///< Distortion coefficients

   /// 按装甲板编号缓存上一帧解 (rvec, tvec)，避免同帧多目标互相污染
   std::map<std::string, std::pair<cv::Mat, cv::Mat>> last_solutions_;

   /// 时序消歧的旋转跳变门限（弧度）：当误差最小解相对上一帧旋转跳变超过此值，
   /// 视为真实跳变（如小陀螺换面），不再用上一帧强行约束，直接取误差最小解。
   static constexpr double MAX_ROT_JUMP = 0.5;  // ~28.6 deg

   /// 时序解的重投影误差相对误差最小解的可接受倍率，超过则判定跟错分支并回退
   static constexpr double REPROJ_FALLBACK_RATIO = 1.5;
 
   // Unit: mm for model points
   static constexpr float SMALL_ARMOR_WIDTH = 135;
   static constexpr float SMALL_ARMOR_HEIGHT = 56;
   static constexpr float LARGE_ARMOR_WIDTH = 230;
   static constexpr float LARGE_ARMOR_HEIGHT = 56;
 
   // 3D model points for different armor types (clockwise order)
   std::vector<cv::Point3f> small_armor_points_;
   /**std::vector<cv::Point3f> outpost_armor_points_; rensy注释**/
   std::vector<cv::Point3f> large_armor_points_;
 
   
   // Intermediate matrices used by algorithms (kept for compatibility)
   // cv::Mat ArmorPos;
   // cv::Mat MuzzlePos;
   // cv::Mat MuzzlePosUse;
 };
 
 }  // namespace rm_auto_aim
 
 #endif  // ARMOR_DETECTOR__PNP_SOLVER_HPP_
 