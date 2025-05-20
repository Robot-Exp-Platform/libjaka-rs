# draw.py
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.colors import Normalize
from matplotlib.cm import ScalarMappable


def load_trajectory(file_path):
    """读取轨迹文件（兼容编码问题）"""
    try:
        with open(file_path, "r", encoding="utf-16") as f:
            data = np.loadtxt(f, delimiter=",")
    except:
        with open(file_path, "rb") as f:
            raw = f.read().decode("utf-16").replace("\x00", "")
        from io import StringIO

        data = np.loadtxt(StringIO(raw), delimiter=",")
    return data[:, :3], data[:, 3:]


def calculate_speeds(xyz, time_step=0.008):
    """计算相邻点之间的速率"""
    displacements = np.linalg.norm(np.diff(xyz, axis=0), axis=1)
    speeds = displacements / time_step
    # 在开头添加一个速度值，使长度与xyz一致
    speeds = np.insert(speeds, 0, speeds[0])
    return speeds


def plot_speed_colored_trajectory(xyz, point_step=5):
    """绘制速度着色的轨迹"""
    fig = plt.figure(figsize=(12, 8))
    ax = fig.add_subplot(111, projection="3d")

    # 计算速度
    speeds = calculate_speeds(xyz)

    # 创建颜色映射
    norm = Normalize(vmin=speeds.min(), vmax=speeds.max())
    cmap = plt.get_cmap("coolwarm")  # 可以使用其他颜色映射如 'plasma', 'inferno'等

    # 绘制速度着色的线段
    for i in range(len(xyz) - 1):
        ax.plot(
            xyz[i : i + 2, 0],
            xyz[i : i + 2, 1],
            xyz[i : i + 2, 2],
            color=cmap(norm(speeds[i])),
            linewidth=2,
            alpha=0.8,
        )

    # 绘制离散点（按步长采样）
    scatter = ax.scatter(
        xyz[::point_step, 0],
        xyz[::point_step, 1],
        xyz[::point_step, 2],
        c=speeds[::point_step],
        cmap=cmap,
        s=20,
        marker="o",
        alpha=0.8,
        label=f"Sampled points (step={point_step})",
    )

    # 标记起点和终点
    ax.scatter(xyz[0, 0], xyz[0, 1], xyz[0, 2], c="g", s=100, marker="*", label="Start")
    ax.scatter(
        xyz[-1, 0], xyz[-1, 1], xyz[-1, 2], c="k", s=100, marker="X", label="End"
    )

    # 设置等比例坐标轴
    set_equal_axes(ax, xyz)

    # 添加颜色条
    sm = ScalarMappable(cmap=cmap, norm=norm)
    sm.set_array([])
    cbar = fig.colorbar(
        sm,
        ax=ax,
        shrink=0.5,
        aspect=10,
        ticks=np.linspace(speeds.min(), speeds.max(), 10),
    )  # 修改此行
    cbar.set_label("Speed (mm/s)", rotation=270, labelpad=15)
    cbar.ax.tick_params(labelsize=8)  # 添加此行，调整刻度标签大小

    ax.set_xlabel("X (mm)")
    ax.set_ylabel("Y (mm)")
    ax.set_zlabel("Z (mm)")
    ax.set_title("Trajectory Colored by Speed")
    ax.legend()
    plt.tight_layout()
    plt.show()


def set_equal_axes(ax, xyz):
    """设置三轴等比例显示"""
    max_range = (
        np.array(
            [
                xyz[:, 0].max() - xyz[:, 0].min(),
                xyz[:, 1].max() - xyz[:, 1].min(),
                xyz[:, 2].max() - xyz[:, 2].min(),
            ]
        ).max()
        * 0.5
    )

    mid_x = (xyz[:, 0].max() + xyz[:, 0].min()) * 0.5
    mid_y = (xyz[:, 1].max() + xyz[:, 1].min()) * 0.5
    mid_z = (xyz[:, 2].max() + xyz[:, 2].min()) * 0.5

    ax.set_xlim(mid_x - max_range, mid_x + max_range)
    ax.set_ylim(mid_y - max_range, mid_y + max_range)
    ax.set_zlim(mid_z - max_range, mid_z + max_range)


if __name__ == "__main__":
    file_path = "scripts/output_spiral"  # 替换为实际路径
    xyz, _ = load_trajectory(file_path)
    plot_speed_colored_trajectory(xyz, point_step=10)  # 每10个点显示一个
