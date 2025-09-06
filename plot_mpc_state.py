import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# 读取数据
file_path = "/home/killer_machine/WRITE/build/mpc_log.txt"
data = np.genfromtxt(file_path, skip_header=1, invalid_raise=False)

# 第一列是曲线1的 y 值，第二列是曲线2的 y 值
y1_vals = data[:, 0]  # 曲线1
y2_vals = data[:, 1]  # 曲线2

# 生成对应的 x 值（假设是时间步或索引）
x_vals = np.arange(len(y1_vals))

# 创建图形
fig, ax = plt.subplots(figsize=(10, 6))
ax.set_xlim(0, len(x_vals))
ax.set_ylim(min(min(y1_vals), min(y2_vals)) - 0.1, 
            max(max(y1_vals), max(y2_vals)) + 0.1)
ax.set_xlabel('Time Step / Index')
ax.set_ylabel('Y Value')
ax.set_title('Two Curves Animation')

# 初始化两条曲线和两个点
line1, = ax.plot([], [], 'b-', label='Curve 1', alpha=0.7)  # 曲线1（蓝色）
line2, = ax.plot([], [], 'r-', label='Curve 2', alpha=0.7)  # 曲线2（红色）
point1, = ax.plot([], [], 'bo', markersize=8)  # 曲线1的当前点
point2, = ax.plot([], [], 'ro', markersize=8)  # 曲线2的当前点

ax.legend()  # 显示图例

# 初始化函数
def init():
    line1.set_data([], [])
    line2.set_data([], [])
    point1.set_data([], [])
    point2.set_data([], [])
    return line1, line2, point1, point2

# 动画更新函数
def update(frame):
    # 更新曲线1
    line1.set_data(x_vals[:frame+1], y1_vals[:frame+1])
    point1.set_data([x_vals[frame]], [y1_vals[frame]])
    
    # 更新曲线2
    line2.set_data(x_vals[:frame+1], y2_vals[:frame+1])
    point2.set_data([x_vals[frame]], [y2_vals[frame]])
    
    return line1, line2, point1, point2

# 创建动画
ani = FuncAnimation(fig, update, frames=len(x_vals),
                    init_func=init, blit=True, interval=100)

plt.tight_layout()
plt.show()
