std_msgs/Header header
float64 raw_yaw           # 直接测量的yaw角（有突变）
float64 cos_obs           # cos(k*yaw) 观测值
float64 sin_obs           # sin(k*yaw) 观测值

float64 ekf_yaw           # EKF估计的yaw
float64 ekf_v_yaw         # EKF估计的角速度（核心对比量）

float64 yaw_recovered     # atan2(sin_obs, cos_obs) / k，应连续无突变

float64 residual_cos      # z(3) - h(x)(3)
float64 residual_sin      # z(4) - h(x)(4)


float64 p_yaw             # P(6,6): yaw不确定度
float64 p_v_yaw           # P(7,7): v_yaw不确定度

int32 k_value             # 当前使用的k值(1/2/3)
int32 match_id

float64 dz1
float64 dz2
float64 ya
float64 xa
float64 za