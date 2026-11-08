import numpy as np
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt

# ==========================================
# 1. 物理参数配置 (根据您提供的规格)
# ==========================================
# 弹丸参数
MASS = 44.5 / 1000       # 质量 (kg) -> 44.5g
DIAMETER = 42.5 / 1000   # 直径 (m) -> 42.5mm
AREA = np.pi * (DIAMETER / 2)**2  # 迎风面积 (m^2)

# 空气动力学参数
# 类似于高尔夫球的凹坑表面，低速(16m/s)下的风阻系数
# 光滑球体约0.5，高尔夫球通常在0.25-0.35之间，这里取保守值0.35
DRAG_COEFF = 0.35        
AIR_DENSITY = 1.225      # 空气密度 (kg/m^3), 15°C海平面标准
GRAVITY = 9.7915        # 重力加速度 (m/s^2) 精确值

def calculate_trajectory(v0, angle_deg, target_distance):
    """
    计算弹丸轨迹并对比有无阻力的差异
    :param v0: 初速度 (m/s)
    :param angle_deg: 仰角 (度)
    :param target_distance: 目标水平距离 (m)
    """
    
    # 将角度转换为弧度
    theta = np.radians(angle_deg)
    
    # 初始速度分量
    vx0 = v0 * np.cos(theta)
    vy0 = v0 * np.sin(theta)
    
    # ---------------------------------------
    # A. 理想情况 (无空气阻力) - 解析解
    # ---------------------------------------
    # 计算飞到目标距离的时间 t = x / vx
    t_ideal = target_distance / vx0
    # 计算高度 y = vy * t - 0.5 * g * t^2
    y_ideal = vy0 * t_ideal - 0.5 * GRAVITY * t_ideal**2
    
    # ---------------------------------------
    # B. 实际情况 (有空气阻力) - 数值解 (RK45)
    # ---------------------------------------
    
    # 定义微分方程组：状态向量 S = [x, y, vx, vy]
    # dS/dt = [vx, vy, ax, ay]
    def equations_of_motion(t, S):
        x, y, vx, vy = S
        
        # 当前速度大小
        v = np.sqrt(vx**2 + vy**2)
        
        # 空气阻力大小 Fd = 0.5 * rho * v^2 * Cd * A
        F_drag = 0.5 * AIR_DENSITY * (v**2) * DRAG_COEFF * AREA
        
        # 阻力分解 (方向与速度相反)
        # Fx = -Fd * (vx / v)
        # Fy = -Fd * (vy / v)
        if v == 0:
            ax = 0
            ay = -GRAVITY
        else:
            ax = -(F_drag * (vx / v)) / MASS
            ay = -GRAVITY - (F_drag * (vy / v)) / MASS
            
        return [vx, vy, ax, ay]

    # 定义终止事件：当水平距离 x 达到 target_distance 时停止计算
    def reach_target(t, S):
        return S[0] - target_distance
    reach_target.terminal = True  # 这是一个终止事件
    reach_target.direction = 1    # 也就是从 x < distance 变成 x > distance

    # 初始状态 [x=0, y=0, vx=vx0, vy=vy0]
    S0 = [0, 0, vx0, vy0]
    
    # 求解微分方程 (时间跨度给大一点，反正会被事件截断)
    # max_step 控制积分步长，越小越精确
    sol = solve_ivp(equations_of_motion, [0, 10], S0, events=reach_target, 
                    dense_output=True, rtol=1e-9, atol=1e-9)
    
    if not sol.t_events[0].size:
        print("警告：弹丸未能到达目标距离（可能速度过低或距离过远）")
        return

    # 获取到达6米那一瞬间的精确状态
    final_time = sol.t_events[0][0]
    final_state = sol.y_events[0][0] # [x, y, vx, vy]
    
    y_drag = final_state[1] # 有阻力时的高度
    
    # ---------------------------------------
    # C. 输出结果
    # ---------------------------------------
    diff_mm = (y_ideal - y_drag) * 1000
    
    print("-" * 40)
    print(f"【参数设定】初速: {v0} m/s | 仰角: {angle_deg}° | 距离: {target_distance} m")
    print("-" * 40)
    print(f"理想高度 (无阻力): {y_ideal:.4f} m")
    print(f"实际高度 (有阻力): {y_drag:.4f} m")
    print(f"高度落差 (Difference): {diff_mm:.2f} mm")
    print("-" * 40)

    # ---------------------------------------
    # D. 绘图 (可视化轨迹)
    # ---------------------------------------
    # 生成平滑的绘图点
    t_plot = np.linspace(0, final_time, 100)
    drag_points = sol.sol(t_plot) # 使用插值函数获取轨迹
    x_drag_plot = drag_points[0]
    y_drag_plot = drag_points[1]
    
    # 计算无阻力的对应轨迹用于绘图对比
    x_ideal_plot = np.linspace(0, target_distance, 100)
    t_ideal_plot = x_ideal_plot / vx0
    y_ideal_plot = vy0 * t_ideal_plot - 0.5 * GRAVITY * t_ideal_plot**2

    plt.figure(figsize=(10, 6))
    plt.plot(x_ideal_plot, y_ideal_plot, '--', label='Ideal (Vacuum)', color='gray', alpha=0.7)
    plt.plot(x_drag_plot, y_drag_plot, '-', label='Actual (With Air Drag)', color='blue', linewidth=2)
    
    # 标记终点
    plt.scatter([target_distance], [y_ideal], color='gray', marker='o')
    plt.scatter([target_distance], [y_drag], color='red', marker='x', s=100, label='Impact Point')
    
    # 画一条垂直线连接两个点，展示高度差
    plt.plot([target_distance, target_distance], [y_ideal, y_drag], color='red', linestyle=':')
    plt.text(target_distance + 0.1, (y_ideal + y_drag)/2, f"Diff: {diff_mm:.1f}mm", color='red')

    plt.title(f"Projectile Trajectory Analysis (Angle: {angle_deg}°)")
    plt.xlabel("Distance (m)")
    plt.ylabel("Height (m)")
    plt.legend()
    plt.grid(True, linestyle='--', alpha=0.6)
    plt.axis('equal') # 保证x和y轴比例一致，视觉更真实
    plt.show()

# ==========================================
# 用户输入区 (您可以修改这里来运行不同参数)
# ==========================================



# 示例 2: 尝试改变参数 (取消注释运行)
# calculate_trajectory(v0=25, angle_deg=15, target_distance=10)

if __name__ == "__main__":
    # 示例 1: 题目中的情况
    calculate_trajectory(v0=11, angle_deg=30, target_distance=8)