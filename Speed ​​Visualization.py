import matplotlib.pyplot as plt
import numpy as np
from matplotlib.colors import LinearSegmentedColormap
import os
from tqdm import tqdm
import cv2

# 仪表盘绘制函数
def create_gauge(current_speed, max_speed=100, avg_speed=50, save_path="rotated_gauge.png"):
    # 设置渐变颜色
    colors = [
    (0.0,   "navy"),       # 深蓝
    (0.15,  "dodgerblue"), # 道奇蓝
    (0.3,   "limegreen"),  # 亮绿
    (0.45,  "yellowgreen"),# 黄绿
    (0.55,  "gold"),       # 金色
    (0.7,   "orange"),     # 橙色
    (0.85,  "red"),        # 红色
    (1.0,   "darkred")     # 深红
    ]

    # 使用颜色字典精细控制渐变
    cdict = {'red':   [], 'green': [], 'blue': []}

    for pos, color in colors:
        r, g, b = plt.cm.colors.to_rgb(color)
        cdict['red'].append(  (pos, r, r) )
        cdict['green'].append((pos, g, g) )
        cdict['blue'].append( (pos, b, b) )

    cmap = LinearSegmentedColormap('smooth_cmap', cdict, N=1024)
    # cmap = LinearSegmentedColormap.from_list("speed_cmap", ["blue","royalblue","cyan","green", "yellow", "red"])
    
    # 创建画布
    fig, ax = plt.subplots(figsize=(6, 6), subplot_kw={'projection': 'polar'})
    ax.set_theta_zero_location('W')  # 将零点旋转到左侧
    ax.set_theta_direction(-1)      # 设置角度方向为顺时针
    ax.set_ylim(0, 1.2)             # 设置径向范围
    ax.set_xticks([])  # 隐藏x轴刻度线
    ax.set_yticks([])  # 隐藏y轴刻度线
    ax.set_xticklabels([])  # 隐藏x轴刻度标签
    ax.set_yticklabels([])  # 隐藏y轴刻度标签

    # 仪表盘的渐变颜色绘制
    theta = np.linspace(-np.pi/6, np.pi + np.pi/6, 500)  # 从 -π/6 到 π + π/6
    norm_speed = current_speed / max_speed
    colors = cmap(np.linspace(0, 1, len(theta)))
    for i in range(len(theta) - 1):
        ax.fill_between(
            [theta[i], theta[i + 1]], 
            0.8, 1, 
            color=colors[i],
            alpha=0.8 if i / len(theta) <= norm_speed else 0
        )
        
    for i in range(len(theta) - 1):
        ax.fill_between(
            [theta[i], theta[i + 1]], 
            1.08, 1.2, 
            color="lightblue"
        )

    # 绘制刻度弧
    for i, angle in enumerate(np.linspace(-np.pi/6, np.pi + np.pi/6, 11)):
        radius = 1.2
        ax.plot([angle, angle], [radius - 0.1, radius], color="white", lw=2)  # 刻度线
       
        speed_label = int(i * max_speed / 10)
        ax.text(angle, radius + 0.1, f"{speed_label}", color="white", fontsize=14, ha="center", va="center")

    # 绘制指针
    pointer_angle = -np.pi/6 + norm_speed * (np.pi+np.pi/3)
    i = np.abs(theta - pointer_angle).argmin()
    
    ax.plot([pointer_angle, pointer_angle], [0.8, 1.25], color=colors[i], lw=3, label="Current Speed")

    # Max Speed 放在右上方
    ax.text(
        -np.pi/2, 0.1,  # 角度π（右侧），半径1.4
        f"Max Vel(Km/H)", 
        color="red", 
        ha='center',   # 右对齐
        va='top',     # 顶部对齐
        fontsize=20,
        fontweight="bold"
    )
    
    ax.text(
        -np.pi/2, 0.3,  # 角度π（右侧），半径1.4
        f"{max_speed:.2f}", 
        color="red", 
        ha='center',   # 右对齐
        va='top',     # 顶部对齐
        fontsize=24,
        fontweight="bold"
    )

    # Current Speed 放在仪表中心下方
    ax.text(
        np.pi/2, 0.30,  # 角度π/2（正上方），半径0.5
        f"Cur Vel(Km/H)", 
        color="cyan", 
        ha='center', 
        va='center' ,
        fontsize=20,
        fontweight="bold"
    )
    
    ax.text(
        np.pi/2, 0.1,  # 角度π/2（正上方），半径0.5
        f"{current_speed:.2f}", 
        color="cyan", 
        ha='center', 
        va='center' ,
        fontsize=24,
        fontweight="bold"
    )


    # 显示轨迹状态
    status = "Safe Traj." if current_speed <= avg_speed + 10 else "Dangerous Traj."
    ax.text(-np.pi/2, 0.6, status, color="green" if status == "Safe Traj." else "red", 
            ha="center", fontsize=20, fontweight="bold", style="italic")

    # 隐藏边框
    ax.spines['polar'].set_visible(False)
    ax.set_facecolor("black")

    # 保存图片
    plt.savefig(save_path, transparent=True, dpi=300)
    plt.close()

# 测试数据
# 生成平滑速度曲线的函数
def generate_smooth_speed(duration=5, fps=30):
    """生成包含加速、巡航、减速阶段的平滑速度曲线"""
    t = np.linspace(0, duration, duration*fps)
    
    # 分段函数参数
    acceleration_duration = duration * 0.4  # 加速阶段30%
    cruise_duration = duration * 0.3         # 巡航阶段40%
    deceleration_duration = duration * 0.4   # 减速阶段30%
    
    # 加速阶段（0-30%时间）
    acceleration = np.linspace(0, 80, int(acceleration_duration*fps))
    
    # 巡航阶段（30-70%时间）
    cruise = np.linspace(80, 70, int(cruise_duration*fps))
    
    # 减速阶段（70-100%时间）
    deceleration = np.linspace(70, 90, int(deceleration_duration*fps))
    
    # 组合三个阶段
    speed = np.concatenate([acceleration, cruise, deceleration])
    
    # 确保长度匹配
    if len(speed) < len(t):
        speed = np.pad(speed, (0, len(t)-len(speed)), mode='edge')
    else:
        speed = speed[:len(t)]
    
    return t, speed

# 视频生成函数
def generate_video(output_path="smooth_speed.mp4", fps=30):
    # 清理并创建临时目录
    os.makedirs("temp_frames", exist_ok=True)
    for f in os.listdir("temp_frames"):
        os.remove(os.path.join("temp_frames", f))
    
    # 生成平滑数据
    t, speeds = generate_smooth_speed(fps=fps)
    
    # 生成所有帧（带进度条）
    for i, speed in enumerate(tqdm(speeds, desc="生成仪表盘帧")):
        create_gauge(
            current_speed=speed,
            max_speed=100,
            avg_speed=50,
            save_path=f"temp_frames/frame_{i:04d}.png"
        )
    
    # 合成视频
    frame_files = sorted([f for f in os.listdir("temp_frames") if f.endswith(".png")])
    
    # 获取第一帧尺寸
    img = cv2.imread(os.path.join("temp_frames", frame_files[0]))
    height, width, _ = img.shape
    
    # 初始化视频写入器
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    video = cv2.VideoWriter(output_path, fourcc, fps, (width, height))
    
    # 写入所有帧
    for frame_file in tqdm(frame_files, desc="合成视频"):
        img = cv2.imread(os.path.join("temp_frames", frame_file))
        video.write(img)
    
    video.release()
    print(f"\n视频生成完成: {output_path}")

if __name__ == "__main__":
    generate_video(fps=30)  # 推荐保持30fps的流畅度