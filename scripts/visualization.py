import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


def visualize_3d_points(csv_file):
    # 读取CSV文件
    data = pd.read_csv(csv_file)

    # 检查数据列
    if not all(col in data.columns for col in ["x", "y", "z"]):
        raise ValueError("CSV文件必须包含x,y,z列")

    # 创建3D图形
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection="3d")

    # 绘制点
    ax.scatter(data["x"], data["y"], data["z"], c="r", marker="o", label="Points")

    # 绘制线（连接点）
    ax.plot(data["x"], data["y"], data["z"], "b-", alpha=0.5, label="Path")

    # 标记第一个和最后一个点
    ax.scatter(
        data["x"].iloc[0],
        data["y"].iloc[0],
        data["z"].iloc[0],
        c="g",
        marker="^",
        s=100,
        label="Start",
    )
    ax.scatter(
        data["x"].iloc[-1],
        data["y"].iloc[-1],
        data["z"].iloc[-1],
        c="k",
        marker="x",
        s=100,
        label="End",
    )

    # 设置坐标轴标签
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")

    # 设置等比例坐标轴
    ax.set_box_aspect([1, 1, 1])  # X:Y:Z比例为1:1:1

    # 自动调整坐标轴范围，使所有点可见且居中
    max_range = (
        max(
            data["x"].max() - data["x"].min(),
            data["y"].max() - data["y"].min(),
            data["z"].max() - data["z"].min(),
        )
        / 2.0
    )

    mid_x = (data["x"].max() + data["x"].min()) * 0.5
    mid_y = (data["y"].max() + data["y"].min()) * 0.5
    mid_z = (data["z"].max() + data["z"].min()) * 0.5

    ax.set_xlim(mid_x - max_range, mid_x + max_range)
    ax.set_ylim(mid_y - max_range, mid_y + max_range)
    ax.set_zlim(mid_z - max_range, mid_z + max_range)

    # 设置标题
    ax.set_title("3D Points Visualization (Equal Aspect Ratio)")

    # 添加图例
    ax.legend()

    # 显示图形
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    import sys

    if len(sys.argv) != 2:
        print("Usage: python visualize_3d.py <csv_file>")
        sys.exit(1)

    csv_file = sys.argv[1]
    visualize_3d_points(csv_file)
