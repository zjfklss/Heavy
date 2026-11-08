import numpy as np
from scipy.integrate import solve_ivp
from scipy.optimize import brentq
import time

# ==========================================
# 1. 物理常数与弹丸属性
# ==========================================
MASS = 0.0445           # kg
DIAMETER = 0.0425       # m
AREA = np.pi * (DIAMETER / 2)**2
DRAG_COEFF = 0.35       
AIR_DENSITY = 1.225     
GRAVITY = 9.7915       

def solve_launch_angle(v0, target_dist, target_height):
    """
    反解仰角 (加入智能区间扫描功能)
    """
    
    # --- 物理模型 ---
    def equations(t, state):
        vx, vy = state[2], state[3]
        v = np.sqrt(vx**2 + vy**2)
        force_drag = 0.5 * AIR_DENSITY * (v**2) * DRAG_COEFF * AREA
        
        if v == 0:
            ax, ay = 0, -GRAVITY
        else:
            ax = -(force_drag * (vx/v)) / MASS
            ay = -GRAVITY - (force_drag * (vy/v)) / MASS
        return [vx, vy, ax, ay]

    # --- 终止事件 ---
    def hit_target_distance(t, state):
        return state[0] - target_dist
    hit_target_distance.terminal = True
    hit_target_distance.direction = 1

    # --- 模拟单次射击，返回(高度差, 飞行时间) ---
    def simulate_shot(angle_rad):
        vx0 = v0 * np.cos(angle_rad)
        vy0 = v0 * np.sin(angle_rad)
        
        # 极大值检查：如果角度太高，水平速度太小，可能永远飞不到目标距离
        # 简单预判：如果水平初速连真空下都飞不到，直接返回失败
        # 真空飞行距离 = vx * (2*vy/g). 这里不做严格限制，交给求解器，但设置最大时间
        if vx0 <= 0.1: 
            return -9999.0, 0
            
        sol = solve_ivp(
            equations, [0, 20], [0, 0, vx0, vy0], # max_step 自动
            events=hit_target_distance, 
            method='DOP853', 
            rtol=1e-12, atol=1e-12
        )
        
        if sol.t_events[0].size == 0:
            # 未到达目标距离（可能落地了，也可能飞不到）
            # 这里我们需要知道它是“落地了但还没到”还是“飞不到”
            # 为了求根函数的连续性，如果飞不到，我们给予一个极大的负惩罚
            return -500.0, 0 
            
        final_y = sol.y_events[0][0][1]
        flight_t = sol.t_events[0][0]
        
        # 返回：当前高度 - 目标高度
        return final_y - target_height, flight_t

    # --- 包装函数 ---
    def error_func(angle):
        diff, _ = simulate_shot(angle)
        return diff

    # --- 2. 智能区间扫描 (核心修复) ---
    # 我们不直接假设 -80 到 +80。我们从低角度往高角度扫。
    # 我们只需要找到一个区间 [a, b]，使得 error(a) 和 error(b) 符号相反。
    # 扫描点：-60度 到 +60度，每隔 10度 扫一次
    scan_points = np.linspace(np.radians(-60), np.radians(65), 15)
    
    bracket_found = False
    low_bound = 0
    high_bound = 0
    
    # 计算第一个点的误差
    prev_angle = scan_points[0]
    prev_err = error_func(prev_angle)
    
    for i in range(1, len(scan_points)):
        curr_angle = scan_points[i]
        curr_err = error_func(curr_angle)
        
        # 检查是否跨越了0点 (符号相反)
        if prev_err * curr_err < 0:
            low_bound = prev_angle
            high_bound = curr_angle
            bracket_found = True
            break # 找到第一个解（低伸弹道解）就停止，忽略高抛解
        
        prev_angle = curr_angle
        prev_err = curr_err
        
    if not bracket_found:
        print("【错误】未找到解区间。可能是初速过低、目标过远或超出物理极限。")
        return None, None

    # --- 3. 精确求根 ---
    try:
        optimal_angle = brentq(error_func, low_bound, high_bound, xtol=1e-15)
    except Exception as e:
        print(f"求解失败: {e}")
        return None, None

    _, final_time = simulate_shot(optimal_angle)
    return optimal_angle, final_time

# ==========================================
# 执行计算
# ==========================================
# 您的特定参数
input_v0 = 15.4
input_dist = 4.75   
input_height = 0.5

print(f"正在计算... (V0={input_v0}m/s, Dist={input_dist}m, Height={input_height}m)")
start_time = time.time()

angle_rad, fly_time = solve_launch_angle(input_v0, input_dist, input_height)

end_time = time.time()

if angle_rad is not None:
    print("="*40)
    print("【计算成功】")
    print(f"需求仰角 (Rad) : {angle_rad:.12f}")
    print(f"需求仰角 (Deg) : {np.degrees(angle_rad):.12f}°")
    print(f"飞行时间 (Time): {fly_time:.12f} s")
    print("-" * 40)
    print(f"计算耗时: {(end_time - start_time)*1000:.2f} ms")
    print("="*40)
