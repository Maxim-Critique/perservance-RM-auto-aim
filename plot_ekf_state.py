import numpy as np
import matplotlib.pyplot as plt
import os
import shutil

# 文件路径配置
file_path = "/home/zhbit/WRITE/build/ekf_data.txt"
output_dir = "/home/zhbit/WRITE/build/plots"  # 图片输出目录

# 清空plots目录
if os.path.exists(output_dir):
    shutil.rmtree(output_dir)
os.makedirs(output_dir, exist_ok=True)  # 重新创建目录
print(f"已清空目录: {output_dir}")

# 读取数据（跳过前19行）
try:
    data = np.loadtxt(file_path, skiprows=9)  # 跳过前19行，从第20行开始读取
    print(f"成功读取 {len(data)} 行数据（从原始文件的第20行开始）")
except Exception as e:
    raise FileNotFoundError(f"数据读取失败: {str(e)}")

# 检查是否有足够的数据
if len(data) == 0:
    raise ValueError("读取的数据为空，请检查文件内容或跳过行数设置")

# 计算累积时间
dt_values = data[:, 0]  # 第一列是时间间隔
cumulative_time = np.cumsum(dt_values)  # 计算累积时间

# 分块处理（每1秒数据一张图）
n_states = 10
target_duration = 5.0  # 每张图的时间跨度（秒）
n_chunks = int(np.ceil(cumulative_time[-1] / target_duration))  # 计算总图数

current_time = 0.0
chunk_idx = 0

while current_time < cumulative_time[-1]:
    # 找到当前时间段的起始和结束索引
    start_idx = np.searchsorted(cumulative_time, current_time)
    end_idx = np.searchsorted(cumulative_time, current_time + target_duration)
    
    # 提取当前时间段的数据
    chunk_data = data[start_idx:end_idx]
    chunk_time = cumulative_time[start_idx:end_idx] - current_time  # 相对时间（0-1秒）
    
    if len(chunk_data) == 0:
        current_time += target_duration
        chunk_idx += 1
        continue
    
    # 提取当前时间段的状态量和条件标记
    states = chunk_data[:, 1:1+n_states]  # 跳过dt列
    conditions = chunk_data[:, -1].astype(int)
    
    # 创建大图（10个子图）
    plt.figure(figsize=(15, 10))
    plt.suptitle(f"EKF States (时间: {current_time:.2f}-{current_time+target_duration:.2f}s)", fontsize=16)
    
    # 绘制每个状态量
    for i in range(n_states):
        plt.subplot(5, 2, i + 1)
        plt.plot(chunk_time, states[:, i], label=f"State {i+1}", linewidth=1)
        
        # 标记条件为1的点
        fire_mask = conditions == 1
        plt.scatter(chunk_time[fire_mask], states[fire_mask, i], 
                   color='red', s=20, label="FIRE")
        
        plt.xlabel("时间 (s)")
        plt.grid(True)
        plt.legend()
    
    # 保存图片
    output_path = os.path.join(output_dir, f"ekf_states_plot_{chunk_idx + 1}.png")
    plt.tight_layout()
    plt.savefig(output_path, dpi=150)
    plt.close()  # 关闭当前图形，避免内存泄漏
    print(f"已保存: {output_path}")
    
    current_time += target_duration
    chunk_idx += 1

# 处理完成后删除数据文件
if os.path.exists(file_path):
    os.remove(file_path)
    print(f"已删除数据文件: {file_path}")

print(f"处理完成，共生成 {n_chunks} 张图。")
