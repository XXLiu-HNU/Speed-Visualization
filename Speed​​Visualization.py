import matplotlib.pyplot as plt
import numpy as np
from matplotlib.colors import LinearSegmentedColormap
from matplotlib.collections import PolyCollection
import os
from tqdm import tqdm
import cv2
import csv
import argparse
from multiprocessing import Pool, cpu_count
import glob
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.colors import LinearSegmentedColormap
from matplotlib.collections import PolyCollection
import os
from tqdm import tqdm
import cv2
import csv
import argparse
from multiprocessing import Pool, cpu_count
import glob

# 预定义配置 ------------------------------------------------------
COLORS = [
    (0.0, "navy"),
    (0.15, "dodgerblue"),
    (0.3, "limegreen"),
    (0.45, "yellowgreen"),
    (0.55, "gold"),
    (0.7, "orange"),
    (0.85, "red"),
    (1.0, "darkred")
]
ANGLES = np.linspace(-np.pi/6, np.pi + np.pi/6, 11)
THETA = np.linspace(-np.pi/6, np.pi + np.pi/6, 500)

# 初始化模板 ------------------------------------------------------
def init_gauge_template():
    """初始化仪表盘模板"""
    fig, ax = plt.subplots(figsize=(6, 6), dpi=300, 
                          subplot_kw={'projection': 'polar'})
    ax.set_theta_zero_location('W')
    ax.set_theta_direction(-1)
    ax.set_ylim(0, 1.2)
    ax.set_xticks([])
    ax.set_yticks([])
    ax.spines['polar'].set_visible(False)
    ax.set_facecolor("black")

    # 静态背景元素
    for i in range(len(THETA)-1):
        ax.fill_between([THETA[i], THETA[i+1]], 1.08, 1.2, color="lightblue")
    
    # 刻度线和标签
    tick_objects = {}
    for idx, angle in enumerate(ANGLES):
        # 刻度线
        line = ax.plot([angle, angle], [1.1, 1.2], color="white", lw=2)[0]
        # 刻度标签
        text = ax.text(angle, 1.3, "", color="white", 
                      fontsize=12, ha='center', va='center')
        tick_objects[f'tick_{idx}'] = text

    # 核心文本元素
    text_objects = {
        'max_label': ax.text(-np.pi/2, 0.1, "Max Vel(m/s)", color="red",
                            fontsize=20, ha='center', va='top', fontweight="bold"),
        'max_value': ax.text(-np.pi/2, 0.3, "", color="red",
                           fontsize=24, ha='center', va='top', fontweight="bold"),
        'cur_label': ax.text(np.pi/2, 0.3, "Cur Vel(m/s)", color="cyan",
                            fontsize=20, ha='center', va='center', fontweight="bold"),
        'cur_value': ax.text(np.pi/2, 0.1, "", color="cyan",
                           fontsize=24, ha='center', va='center', fontweight="bold"),
        'status': ax.text(-np.pi/2, 0.6, "", fontsize=20,
                         ha='center', fontweight="bold", style="italic")
    }
    text_objects.update(tick_objects)

    # 颜色渐变区域
    verts = []
    colors = cmap(np.linspace(0, 1, len(THETA)-1))
    for i in range(len(THETA)-1):
        verts.append([(THETA[i], 0.8), (THETA[i], 1), 
                     (THETA[i+1], 1), (THETA[i+1], 0.8)])
    poly = PolyCollection(verts, facecolors=colors, alpha=0)
    ax.add_collection(poly)

    # 指针
    pointer = ax.plot([0, 0], [0.8, 1.25], color=cmap(0), lw=3)[0]

    plt.close(fig)
    return fig, ax, poly, pointer, text_objects

# 全局初始化
cdict = {'red': [], 'green': [], 'blue': []}
for pos, color in COLORS:
    r, g, b = plt.cm.colors.to_rgb(color)
    cdict['red'].append((pos, r, r))
    cdict['green'].append((pos, g, g))
    cdict['blue'].append((pos, b, b))
cmap = LinearSegmentedColormap('smooth_cmap', cdict, N=1024)
FIG, AX, POLY, POINTER, TEXTS = init_gauge_template()

# 核心功能 ------------------------------------------------------
def create_gauge_optimized(args):
    """优化后的仪表盘生成函数"""
    i, speed, max_speed, avg_speed = args
    save_path = f"temp_frames/frame_{i:04d}.png"
    
    # 计算动态参数
    norm_speed = speed / max_speed
    pointer_angle = -np.pi/6 + norm_speed * (np.pi + np.pi/3)
    
    # 更新颜色区域
    alphas = np.zeros(len(THETA)-1)
    active_idx = int(norm_speed * (len(THETA)-1))
    alphas[:active_idx] = 0.8
    POLY.set_alpha(alphas)
    
    # 更新指针
    POINTER.set_xdata([pointer_angle, pointer_angle])
    POINTER.set_color(cmap(norm_speed))
    
    # 更新文本
    TEXTS['max_value'].set_text(f"{max_speed:.2f}")
    TEXTS['cur_value'].set_text(f"{speed:.2f}")
    status = "Safe Traj." if speed <= avg_speed + 0.2*avg_speed else "Dangerous Traj."
    TEXTS['status'].set_text(status)
    TEXTS['status'].set_color("green" if "Safe" in status else "red")
    
    # 更新刻度标签
    for idx in range(len(ANGLES)):
        label_value = round(idx * max_speed / (len(ANGLES)-1), 1)
        TEXTS[f'tick_{idx}'].set_text(f"{label_value:.1f}")

    FIG.savefig(save_path, bbox_inches='tight', transparent=True, dpi=300)
    return save_path

# 数据读取和视频生成 ----------------------------------------------
def read_speed_data(csv_path):
    """优化数据读取"""
    with open(csv_path) as f:
        reader = csv.DictReader(f)
        return [float(row['speed_magnitude']) for row in reader 
               if row['speed_magnitude'].replace('.','',1).isdigit()]

def generate_video_optimized(csv_path, output_path="output.mp4", fps=30):
    """并行视频生成"""
    # 准备数据
    speeds = read_speed_data(csv_path)
    max_speed = max(speeds) * 1.1
    avg_speed = np.mean(speeds)
    
    # 清理临时目录
    os.makedirs("temp_frames", exist_ok=True)
    [os.remove(f) for f in glob.glob("temp_frames/*.png") if os.path.exists(f)]
    
    # 并行生成帧
    with Pool(cpu_count()) as pool:
        params = [(i, s, max_speed, avg_speed) 
                 for i, s in enumerate(speeds)]
        list(tqdm(pool.imap(create_gauge_optimized, params),
                 total=len(speeds), desc="生成帧"))

    # 合成视频
    frame_files = sorted(glob.glob("temp_frames/*.png"))
    h, w = cv2.imread(frame_files[0]).shape[:2]
    
    writer = cv2.VideoWriter(output_path, 
                           cv2.VideoWriter_fourcc(*'mp4v'), 
                           fps, (w, h))
    
    for f in tqdm(frame_files, desc="写入视频"):
        writer.write(cv2.imread(f))
    writer.release()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='无人机速度可视化生成器')
    parser.add_argument('--csv', required=True, help='输入CSV文件路径')
    parser.add_argument('--output', default='output.mp4', help='输出视频路径')
    parser.add_argument('--fps', type=int, default=30, help='视频帧率')
    args = parser.parse_args()
    
    generate_video_optimized(
        csv_path=args.csv,
        output_path=args.output,
        fps=args.fps
    )