import matplotlib.pyplot as plt
import numpy as np
from matplotlib.colors import LinearSegmentedColormap
import os
from tqdm import tqdm
import cv2
import rospy
import csv

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
       
        speed_label = round(i * max_speed / 10, 1)
        ax.text(angle, radius + 0.2, 
            f"{speed_label:.1f}",  # 强制显示一位小数格式
            color="white", 
            fontsize=16, 
            ha="center", 
            va="center")

    # 绘制指针
    pointer_angle = -np.pi/6 + norm_speed * (np.pi+np.pi/3)
    i = np.abs(theta - pointer_angle).argmin()
    
    ax.plot([pointer_angle, pointer_angle], [0.8, 1.25], color=colors[i], lw=3, label="Current Speed")

    # Max Speed 放在右上方
    ax.text(
        -np.pi/2, 0.1,  # 角度π（右侧），半径1.4
        f"Max Vel(m/s)", 
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
        f"Cur Vel(m/s)", 
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

# 从CSV读取速度数据
def read_speed_data(csv_path):
    speeds = []
    with open(csv_path, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                # 添加绝对值处理
                speed = float(row['speed_magnitude'])
                speeds.append(speed)
            except KeyError:
                raise Exception("CSV文件中缺少'speed_magnitude'列, 请确保使用正确版本的odom_recorder")
            except ValueError as e:
                print(f"数据格式错误: {e}，跳过无效行")
    return speeds

# 视频生成函数（修改版）
def generate_video(csv_path, output_path="speed_visualization.mp4", fps=30):
    # 清理并创建临时目录
    os.makedirs("temp_frames", exist_ok=True)
    for f in os.listdir("temp_frames"):
        os.remove(os.path.join("temp_frames", f))
    
    # 读取速度数据
    speeds = read_speed_data(csv_path)
    
    if not speeds:
        raise ValueError("CSV文件中未找到有效速度数据")
    
    # 自动计算最大和平均速度
    max_speed = max(speeds)
    avg_speed = np.mean(speeds)
    
    rospy.loginfo(f"数据统计 - 最大速度: {max_speed:.2f} m/s, 平均速度: {avg_speed:.2f} m/s")
    
    # 生成所有帧（带进度条）
    for i, speed in enumerate(tqdm(speeds, desc="生成仪表盘帧")):
        create_gauge(
            current_speed=speed,
            max_speed=max_speed * 1.1,  # 留10%余量
            avg_speed=avg_speed,
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
    import argparse
    parser = argparse.ArgumentParser(description='生成速度可视化视频')
    parser.add_argument('--csv', type=str, default='odom_data.csv', help='输入CSV文件路径')
    parser.add_argument('--output', type=str, default='speed_visualization.mp4', help='输出视频路径')
    parser.add_argument('--fps', type=int, default=30, help='视频帧率')
    args = parser.parse_args()
    
    generate_video(
        csv_path=args.csv,
        output_path=args.output,
        fps=args.fps
    )