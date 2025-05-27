import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial.transform import Rotation as R

# 参数设置（与Rust代码完全一致）
tool_offset = np.array([107.0, 0.0, 30.0])  # 工具偏移量
vertex = np.array([250.0, 0.0, 30.0])      # 圆锥顶点
h = 60.0                                   # 圆锥高度
loops = 3                                  # 螺旋圈数
theta = 0.1                                # 圆锥半角
alpha = 0.1                                # 工具倾角

def compute_trajectories(steps=100):
    t_values = np.linspace(0, 1, steps)
    tool_positions = []
    tool_rotations = []  # 存储Rotation对象
    flange_positions = []
    
    for t in t_values:
        x = np.sqrt(t)  # 与Rust代码一致的进度控制
        
        # 1. 计算工具末端位置（与compute_point一致）
        radius = h * x * np.tan(theta)
        angle = 2 * np.pi * loops * x
        tool_pos = np.array([
            vertex[0] + radius * np.cos(angle),
            vertex[1] + radius * np.sin(angle),
            vertex[2] + h * x
        ])
        
        # 2. 计算工具姿态（与compute_point一致）
        radial = np.array([vertex[0]-tool_pos[0], vertex[1]-tool_pos[1], 0])
        radial = radial / (np.linalg.norm(radial) + 1e-6)
        
        z_tool = np.array([0, 0, -1]) * np.cos(alpha) + radial * np.sin(alpha)
        z_tool = z_tool / np.linalg.norm(z_tool)
        
        up = np.array([0, 1, 0]) if abs(z_tool[2]) < 0.99 else np.array([1, 0, 0])
        x_dir = np.cross(up, z_tool)
        x_dir = x_dir / (np.linalg.norm(x_dir) + 1e-6)
        y_dir = np.cross(z_tool, x_dir)
        
        rot_matrix = np.column_stack([x_dir, y_dir, z_tool])
        rotation = R.from_matrix(rot_matrix)
        
        # 3. 计算法兰位置（与闭包转换逻辑一致）
        flange_pos = tool_pos - rotation.apply(tool_offset)
        
        # 存储结果
        tool_positions.append(tool_pos)
        tool_rotations.append(rotation)
        flange_positions.append(flange_pos)
    
    return np.array(tool_positions), tool_rotations, np.array(flange_positions)

def visualize_trajectories(tool_pos, tool_rots, flange_pos):
    fig = plt.figure(figsize=(14, 10))
    ax = fig.add_subplot(111, projection='3d')
    
    # 绘制工具轨迹（红色）
    ax.plot(tool_pos[:,0], tool_pos[:,1], tool_pos[:,2], 
            'r-', linewidth=3, label='Tool Path (End Effector)')
    
    # 绘制法兰轨迹（蓝色）
    ax.plot(flange_pos[:,0], flange_pos[:,1], flange_pos[:,2], 
            'b--', linewidth=2, label='Flange Path')
    
    # 绘制关键帧的工具坐标系
    frame_indices = [0, len(tool_pos)//2, -1]  # 显示首、中、末三个关键帧
    for i in frame_indices:
        # 工具坐标系
        draw_frame(ax, tool_pos[i], tool_rots[i], color='r', scale=30)
        # 法兰坐标系（工具坐标系减去偏移量后的姿态）
        draw_frame(ax, flange_pos[i], tool_rots[i], color='b', scale=30)
        # 连接线
        ax.plot([tool_pos[i,0], flange_pos[i,0]], 
                [tool_pos[i,1], flange_pos[i,1]], 
                [tool_pos[i,2], flange_pos[i,2]], 
                'g:', alpha=0.8)

    # 设置图形参数
    ax.set_xlabel('X (mm)')
    ax.set_ylabel('Y (mm)')
    ax.set_zlabel('Z (mm)')
    ax.set_title('Tool vs Flange Trajectory Visualization\n(Red: Tool, Blue: Flange)')
    ax.legend()
    ax.grid(True)
    
    # 等比例显示
    set_axes_equal(ax)
    plt.tight_layout()
    plt.show()

def draw_frame(ax, position, rotation, color='r', scale=20):
    """绘制3D坐标系指示器"""
    rot_mat = rotation.as_matrix()
    for i, col in enumerate(['r', 'g', 'b']):
        ax.quiver(position[0], position[1], position[2],
                 rot_mat[0,i]*scale, rot_mat[1,i]*scale, rot_mat[2,i]*scale,
                 color=col, arrow_length_ratio=0.1, linewidth=2)

def set_axes_equal(ax):
    """使3D坐标轴等比例"""
    limits = np.array([
        ax.get_xlim3d(),
        ax.get_ylim3d(),
        ax.get_zlim3d(),
    ])
    origin = np.mean(limits, axis=1)
    radius = 0.5 * np.max(np.abs(limits[:,1] - limits[:,0]))
    ax.set_xlim3d([origin[0] - radius, origin[0] + radius])
    ax.set_ylim3d([origin[1] - radius, origin[1] + radius])
    ax.set_zlim3d([origin[2] - radius, origin[2] + radius])

# 执行计算和可视化
tool_pos, tool_rots, flange_pos = compute_trajectories(steps=200)
visualize_trajectories(tool_pos, tool_rots, flange_pos)