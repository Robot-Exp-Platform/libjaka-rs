import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def load_trajectory(file_path):
    """读取轨迹文件（兼容编码问题）"""
    try:
        with open(file_path, 'r', encoding='utf-16') as f:
            data = np.loadtxt(f, delimiter=',')
    except:
        with open(file_path, 'rb') as f:
            raw = f.read().decode('utf-16').replace('\x00', '')
        from io import StringIO
        data = np.loadtxt(StringIO(raw), delimiter=',')
    return data[:, :3], data[:, 3:]

def plot_discrete_trajectory(xyz, point_step=5):
    """绘制离散点+折线连接的轨迹"""
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    # 绘制折线（浅色半透明）
    ax.plot(xyz[:, 0], xyz[:, 1], xyz[:, 2], 
            'b-', alpha=0.3, linewidth=1, label='Path')
    
    # 绘制离散点（按步长采样）
    ax.scatter(xyz[::point_step, 0], xyz[::point_step, 1], xyz[::point_step, 2], 
               c='r', s=20, marker='o', alpha=0.8, label=f'Points (step={point_step})')
    
    # 标记起点和终点
    ax.scatter(xyz[0, 0], xyz[0, 1], xyz[0, 2], 
               c='g', s=100, marker='*', label='Start')
    ax.scatter(xyz[-1, 0], xyz[-1, 1], xyz[-1, 2], 
               c='k', s=100, marker='X', label='End')
    
    # 设置等比例坐标轴
    set_equal_axes(ax, xyz)
    
    ax.set_xlabel('X (mm)')
    ax.set_ylabel('Y (mm)')
    ax.set_zlabel('Z (mm)')
    ax.set_title('Discrete Trajectory Points with Connections')
    ax.legend()
    plt.tight_layout()
    plt.show()

def set_equal_axes(ax, xyz):
    """设置三轴等比例显示"""
    max_range = np.array([
        xyz[:, 0].max()-xyz[:, 0].min(), 
        xyz[:, 1].max()-xyz[:, 1].min(), 
        xyz[:, 2].max()-xyz[:, 2].min()
    ]).max() * 0.5
    
    mid_x = (xyz[:, 0].max()+xyz[:, 0].min()) * 0.5
    mid_y = (xyz[:, 1].max()+xyz[:, 1].min()) * 0.5
    mid_z = (xyz[:, 2].max()+xyz[:, 2].min()) * 0.5
    
    ax.set_xlim(mid_x - max_range, mid_x + max_range)
    ax.set_ylim(mid_y - max_range, mid_y + max_range)
    ax.set_zlim(mid_z - max_range, mid_z + max_range)

if __name__ == "__main__":
    file_path = "scripts/output_3_10"  # 替换为实际路径
    xyz, _ = load_trajectory(file_path)
    plot_discrete_trajectory(xyz, point_step=10)  # 每10个点显示一个
