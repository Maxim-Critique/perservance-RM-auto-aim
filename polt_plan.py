import matplotlib.pyplot as plt
import numpy as np
import re
import time
import matplotlib
import sys

class PolynomialVisualizer:
    def __init__(self, log_file):
        self.log_file = log_file
        self.data = self.parse_log_file()
        
    def parse_log_file(self):
        """解析日志文件，提取关键信息"""
        data = {
            'path_points': [],
            'left_extremum': None,
            'right_extremum': None,
            'center_index': None,
            'left_poly_search': [],
            'right_poly_search': [],
            'selected_poly': None
        }
        
        try:
            with open(self.log_file, 'r', encoding='utf-8') as f:
                lines = f.readlines()
            print(f"成功读取日志文件，共 {len(lines)} 行")
        except FileNotFoundError:
            print(f"错误：找不到文件 {self.log_file}")
            return data
        except Exception as e:
            print(f"读取文件时出错: {e}")
            return data
            
        i = 0
        while i < len(lines):
            line = lines[i].strip()
            
            # 解析路径点
            if line.startswith('路径点'):
                match = re.search(r'路径点 (\d+): 时间=([\d.-]+), 角度=([\d.-]+)', line)
                if match:
                    idx, time, angle = match.groups()
                    data['path_points'].append({
                        'index': int(idx),
                        'time': float(time),
                        'angle': float(angle)
                    })
            
            # 解析中心点
            elif '中心点索引' in line:
                match = re.search(r'中心点索引: (\d+)', line)
                if match:
                    data['center_index'] = int(match.group(1))
            
            # 解析极值点
            elif '找到左侧极值点' in line:
                match = re.search(r'索引=(\d+)', line)
                if match:
                    data['left_extremum'] = int(match.group(1))
            elif '找到右侧极值点' in line:
                match = re.search(r'索引=(\d+)', line)
                if match:
                    data['right_extremum'] = int(match.group(1))
            
            # 解析多项式搜索过程
            elif line.startswith('迭代'):
                # 判断是左侧还是右侧极值点的搜索
                current_extremum = 'left'
                # 检查前几行确定是左侧还是右侧
                for j in range(max(0, i-3), i):
                    if '右侧极值点' in lines[j] or 'right_extremum' in lines[j].lower():
                        current_extremum = 'right'
                        break
                    elif '左侧极值点' in lines[j] or 'left_extremum' in lines[j].lower():
                        current_extremum = 'left'
                        break
                    
                match = re.search(r'迭代 (\d+): 使用点 \[(\d+)\] 和 \[(\d+)\]', line)
                if match:
                    iteration, start_idx, end_idx = match.groups()
                    poly_coeffs = []
                    
                    # 查找多项式系数
                    j = i + 1
                    while j < len(lines) and '多项式系数:' not in lines[j]:
                        j += 1
                    
                    if j < len(lines):
                        coeff_line = lines[j]
                        coeffs = re.findall(r'[-+]?\d*\.\d+|\d+', coeff_line.split(':', 1)[1])
                        poly_coeffs = [float(c) for c in coeffs]
                    
                    search_data = {
                        'iteration': int(iteration),
                        'start_idx': int(start_idx),
                        'end_idx': int(end_idx),
                        'coefficients': poly_coeffs,
                        'status': 'checking',
                        'extremum_type': current_extremum
                    }
                    
                    # 检查是否通过约束
                    k = j + 1
                    while k < len(lines) and k < j + 5:
                        if '加速度约束检查: 通过' in lines[k]:
                            search_data['status'] = 'passed'
                            # 检查距离条件
                            if '距离条件满足' in lines[k+1] or '找到有效多项式' in lines[k+1]:
                                search_data['status'] = 'selected'
                            break
                        elif '加速度约束检查: 未通过' in lines[k]:
                            search_data['status'] = 'failed'
                            break
                        k += 1
                    
                    if current_extremum == 'left':
                        data['left_poly_search'].append(search_data)
                    else:
                        data['right_poly_search'].append(search_data)
            
            i += 1
        
        print(f"解析完成: 路径点 {len(data['path_points'])} 个, "
              f"左侧搜索 {len(data['left_poly_search'])} 次, "
              f"右侧搜索 {len(data['right_poly_search'])} 次")
        
        return data
    
    def evaluate_polynomial(self, coeffs, t):
        """计算多项式在时间t处的值"""
        if not coeffs or len(coeffs) < 6:
            return 0
        return (coeffs[0] + coeffs[1]*t + coeffs[2]*t**2 + 
                coeffs[3]*t**3 + coeffs[4]*t**4 + coeffs[5]*t**5)
    
    def debug_data(self):
        """调试函数，显示解析的数据"""
        print("\n=== 调试信息 ===")
        print(f"路径点数量: {len(self.data['path_points'])}")
        print(f"中心点索引: {self.data['center_index']}")
        print(f"左侧极值点索引: {self.data['left_extremum']}")
        print(f"右侧极值点索引: {self.data['right_extremum']}")
        print(f"左侧搜索次数: {len(self.data['left_poly_search'])}")
        print(f"右侧搜索次数: {len(self.data['right_poly_search'])}")
        
        if self.data['path_points']:
            print(f"时间范围: {self.data['path_points'][0]['time']} 到 {self.data['path_points'][-1]['time']}")
            print(f"角度范围: {min(p['angle'] for p in self.data['path_points'])} 到 {max(p['angle'] for p in self.data['path_points'])}")
        
        # 显示搜索详情
        print("\n=== 左侧搜索详情 ===")
        for i, search in enumerate(self.data['left_poly_search']):
            print(f"迭代 {search['iteration']}: 点[{search['start_idx']}]到点[{search['end_idx']}], 状态: {search['status']}")
    
    def interactive_animation(self):
        """交互式动画显示"""
        print("开始交互式动画显示...")
        print("按任意键继续到下一帧，按 'q' 退出")
        
        # 检查数据是否有效
        if not self.data['path_points']:
            print("错误：没有有效的路径点数据")
            return
        
        # 提取路径数据
        times = [p['time'] for p in self.data['path_points']]
        angles = [p['angle'] for p in self.data['path_points']]
        
        # 合并搜索过程
        all_searches = self.data['left_poly_search'] + self.data['right_poly_search']
        
        # 设置交互模式
        plt.ion()
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(14, 10))
        
        # 初始帧
        self.clear_axes(ax1, ax2)
        self.plot_initial_frame(ax1, ax2, times, angles, "初始状态 - 准备开始搜索", 0, len(all_searches)*2+2)
        plt.tight_layout()
        plt.show()
        
        if not self.wait_for_input():
            plt.close()
            return
        
        # 遍历所有搜索
        frame_count = 1
        for search_idx, search in enumerate(all_searches):
            # 第一帧：显示输入点
            self.clear_axes(ax1, ax2)
            self.plot_search_frame(ax1, ax2, times, angles, search, 'show_points', 
                                frame_count, len(all_searches)*2+2, all_searches[:search_idx])
            plt.tight_layout()
            plt.draw()
            
            print(f"显示帧 {frame_count}: {search['extremum_type']}极值点 - 迭代 {search['iteration']} - 选择输入点")
            if not self.wait_for_input():
                break
            frame_count += 1
            
            # 第二帧：显示多项式拟合
            self.clear_axes(ax1, ax2)
            self.plot_search_frame(ax1, ax2, times, angles, search, 'show_polynomial', 
                                frame_count, len(all_searches)*2+2, all_searches[:search_idx])
            plt.tight_layout()
            plt.draw()
            
            print(f"显示帧 {frame_count}: {search['extremum_type']}极值点 - 迭代 {search['iteration']} - 多项式拟合")
            if not self.wait_for_input():
                break
            frame_count += 1
        
        # 最终总结帧
        if frame_count <= len(all_searches)*2+1:
            self.clear_axes(ax1, ax2)
            self.plot_final_summary(ax1, ax2, times, angles, frame_count, len(all_searches)*2+2)
            plt.tight_layout()
            plt.draw()
            
            print(f"显示帧 {frame_count}: 最终总结")
            self.wait_for_input()
        
        plt.ioff()
        plt.close()
        print("动画显示结束")
    
    def wait_for_input(self):
        """等待用户输入"""
        print("按任意键继续，按 'q' 退出...")
        try:
            # 使用plt的等待功能
            if not plt.waitforbuttonpress(0):
                return True
            # 检查是否按了q键
            event = plt.gcf().canvas.key_press_event
            if event and event.key == 'q':
                return False
            return True
        except:
            # 如果上面的方法失败，使用简单的输入
            user_input = input().strip().lower()
            return user_input != 'q'
    
    def clear_axes(self, ax1, ax2):
        """清空坐标轴"""
        ax1.clear()
        ax2.clear()
    
    def plot_initial_frame(self, ax1, ax2, times, angles, title, frame_idx, total_frames):
        """绘制初始帧"""
        # 第一个子图：路径和点
        ax1.plot(times, angles, 'b-', linewidth=3, alpha=0.8, label='原始路径')
        ax1.scatter(times, angles, c='gray', s=40, alpha=0.7, label='路径点')
        
        # 标记特殊点
        self.mark_special_points(ax1)
        
        ax1.set_xlabel('时间 (s)', fontsize=12)
        ax1.set_ylabel('角度 (rad)', fontsize=12)
        ax1.set_title(f'{title}\n进度: {frame_idx+1}/{total_frames}', fontsize=14, fontweight='bold')
        ax1.legend()
        ax1.grid(True, alpha=0.4)
        
        # 第二个子图：说明
        ax2.text(0.5, 0.5, "多项式搜索算法演示\n\n将展示左右极值点的多项式拟合过程", 
                transform=ax2.transAxes, fontsize=16, horizontalalignment='center', 
                verticalalignment='center', bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.9))
        ax2.set_xlim(0, 1)
        ax2.set_ylim(0, 1)
        ax2.axis('off')
    
    def plot_search_frame(self, ax1, ax2, times, angles, search, frame_type, frame_idx, total_frames, completed_searches):
        """绘制搜索帧"""
        colors = {'left': 'blue', 'right': 'red'}
        color = colors[search['extremum_type']]
        
        # 第一个子图：完整路径和已完成搜索
        ax1.plot(times, angles, 'k-', linewidth=2, alpha=0.5, label='原始路径')
        ax1.scatter(times, angles, c='lightgray', s=30, alpha=0.6, label='路径点')
        
        # 标记特殊点
        self.mark_special_points(ax1)
        
        # 显示已完成的多项式
        for comp_search in completed_searches:
            if comp_search['coefficients']:
                comp_color = colors[comp_search['extremum_type']]
                start_point = self.data['path_points'][comp_search['start_idx']]
                end_point = self.data['path_points'][comp_search['end_idx']]
                t_range = np.linspace(start_point['time'], end_point['time'], 100)
                poly_values = [self.evaluate_polynomial(comp_search['coefficients'], t) for t in t_range]
                
                linestyle = '--' if comp_search['status'] == 'passed' else ':'
                if comp_search['status'] == 'selected':
                    linestyle = '-'
                
                ax1.plot(t_range, poly_values, color=comp_color, linestyle=linestyle, 
                        linewidth=2, alpha=0.7, label=f"{comp_search['extremum_type']}-{comp_search['iteration']}")
        
        ax1.set_xlabel('时间 (s)', fontsize=12)
        ax1.set_ylabel('角度 (rad)', fontsize=12)
        ax1.set_title(f'完整路径 - 进度: {frame_idx+1}/{total_frames}', fontsize=14, fontweight='bold')
        ax1.legend(fontsize=8)
        ax1.grid(True, alpha=0.4)
        
        # 第二个子图：当前搜索
        ax2.plot(times, angles, 'k-', linewidth=1, alpha=0.3, label='原始路径')
        
        # 获取相关点
        start_point = self.data['path_points'][search['start_idx']]
        end_point = self.data['path_points'][search['end_idx']]
        extremum_point = self.data['path_points'][
            self.data['left_extremum'] if search['extremum_type'] == 'left' 
            else self.data['right_extremum']
        ]
        
        if frame_type == 'show_points':
            # 显示输入点
            ax2.scatter([start_point['time'], end_point['time']],
                       [start_point['angle'], end_point['angle']],
                       color=color, s=150, zorder=10, 
                       marker='o', edgecolors='black', linewidth=2,
                       label='输入点')
            
            ax2.scatter([extremum_point['time']], [extremum_point['angle']],
                       color=color, s=120, zorder=10, marker='s',
                       edgecolors='black', linewidth=1.5,
                       label=f'{search["extremum_type"]}极值点')
            
            # 连接线
            ax2.plot([start_point['time'], end_point['time']],
                    [start_point['angle'], end_point['angle']],
                    color=color, linestyle='--', alpha=0.7, linewidth=2)
            
            ax2.set_title(f'选择输入点: {search["extremum_type"]}极值点 - 迭代 {search["iteration"]}', 
                         fontsize=14, fontweight='bold')
            
            info_text = (f"起点: 点[{search['start_idx']}], 时间={start_point['time']:.3f}, 角度={start_point['angle']:.3f}\n"
                        f"终点: 点[{search['end_idx']}], 时间={end_point['time']:.3f}, 角度={end_point['angle']:.3f}\n"
                        f"极值点: 点[{self.data['left_extremum' if search['extremum_type'] == 'left' else 'right_extremum']}]")
            
            ax2.text(0.02, 0.98, info_text, transform=ax2.transAxes, fontsize=9,
                    verticalalignment='top', bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.9))
            
        else:  # show_polynomial
            # 显示多项式
            if search['coefficients']:
                t_range = np.linspace(start_point['time'], end_point['time'], 100)
                poly_values = [self.evaluate_polynomial(search['coefficients'], t) for t in t_range]
                
                ax2.plot(t_range, poly_values, color=color, linewidth=3, 
                        label='拟合多项式', alpha=0.9)
                
                ax2.scatter([start_point['time'], end_point['time']],
                           [start_point['angle'], end_point['angle']],
                           color=color, s=100, zorder=10, 
                           marker='o', edgecolors='black', linewidth=1.5,
                           label='输入点')
                
                ax2.set_title(f'多项式拟合: {search["extremum_type"]}极值点 - 迭代 {search["iteration"]}', 
                            fontsize=14, fontweight='bold')
                
                status_text = "通过约束 ✓" if search['status'] in ['passed', 'selected'] else "未通过约束 ✗"
                if search['status'] == 'selected':
                    status_text = "选中多项式 ★"
                
                result_text = (f"多项式阶数: 5阶\n"
                             f"约束检查: {status_text}\n"
                             f"起点索引: {search['start_idx']}\n"
                             f"终点索引: {search['end_idx']}")
                
                box_color = 'lightgreen' if search['status'] in ['passed', 'selected'] else 'lightcoral'
                ax2.text(0.02, 0.98, result_text, transform=ax2.transAxes, fontsize=10,
                        verticalalignment='top', bbox=dict(boxstyle='round', facecolor=box_color, alpha=0.9))
        
        ax2.set_xlabel('时间 (s)', fontsize=12)
        ax2.set_ylabel('角度 (rad)', fontsize=12)
        ax2.legend()
        ax2.grid(True, alpha=0.4)
    
    def plot_final_summary(self, ax1, ax2, times, angles, frame_idx, total_frames):
        """绘制最终总结帧"""
        # 第一个子图：显示所有搜索
        ax1.plot(times, angles, 'k-', linewidth=2, alpha=0.5, label='原始路径')
        self.mark_special_points(ax1)
        
        colors = {'left': 'blue', 'right': 'red'}
        for search in self.data['left_poly_search'] + self.data['right_poly_search']:
            if search['coefficients']:
                color = colors[search['extremum_type']]
                start_point = self.data['path_points'][search['start_idx']]
                end_point = self.data['path_points'][search['end_idx']]
                t_range = np.linspace(start_point['time'], end_point['time'], 100)
                poly_values = [self.evaluate_polynomial(search['coefficients'], t) for t in t_range]
                
                linestyle = '--' if search['status'] == 'passed' else ':'
                if search['status'] == 'selected':
                    linestyle = '-'
                
                ax1.plot(t_range, poly_values, color=color, linestyle=linestyle, 
                        linewidth=2, alpha=0.7, label=f"{search['extremum_type']}-{search['iteration']}-{search['status']}")
        
        ax1.set_title('所有搜索过程汇总', fontsize=14, fontweight='bold')
        ax1.legend(fontsize=7)
        ax1.grid(True, alpha=0.4)
        
        # 第二个子图：统计信息
        selected_left = len([s for s in self.data['left_poly_search'] if s['status'] == 'selected'])
        selected_right = len([s for s in self.data['right_poly_search'] if s['status'] == 'selected'])
        
        result_text = (f"搜索完成！\n\n"
                      f"总搜索次数: {len(self.data['left_poly_search']) + len(self.data['right_poly_search'])}\n"
                      f"左侧搜索: {len(self.data['left_poly_search'])} 次\n"
                      f"右侧搜索: {len(self.data['right_poly_search'])} 次\n"
                      f"选中多项式: 左侧 {selected_left} 个, 右侧 {selected_right} 个\n\n"
                      f"动画结束")
        
        ax2.text(0.5, 0.5, result_text, transform=ax2.transAxes, fontsize=14,
                horizontalalignment='center', verticalalignment='center',
                bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.9))
        ax2.set_xlim(0, 1)
        ax2.set_ylim(0, 1)
        ax2.axis('off')
    
    def mark_special_points(self, ax):
        """标记特殊点"""
        if self.data['center_index'] is not None:
            center_point = self.data['path_points'][self.data['center_index']]
            ax.scatter(center_point['time'], center_point['angle'], 
                      color='red', s=200, marker='*', label='中心点', zorder=10,
                      edgecolors='black', linewidth=2)
        
        if self.data['left_extremum'] is not None:
            left_point = self.data['path_points'][self.data['left_extremum']]
            ax.scatter(left_point['time'], left_point['angle'], 
                      color='green', s=150, marker='^', label='左侧极值点', zorder=10,
                      edgecolors='black', linewidth=2)
        
        if self.data['right_extremum'] is not None:
            right_point = self.data['path_points'][self.data['right_extremum']]
            ax.scatter(right_point['time'], right_point['angle'], 
                      color='orange', s=150, marker='v', label='右侧极值点', zorder=10,
                      edgecolors='black', linewidth=2)

def main():
    # 使用示例
    log_file = '/home/killer_machine/WRITE/build/planner_log.txt'
    
    print(f"尝试读取日志文件: {log_file}")
    visualizer = PolynomialVisualizer(log_file)
    
    # 显示调试信息
    visualizer.debug_data()
    
    # 询问用户选择哪种显示方式
    print("\n请选择显示方式:")
    print("1. 交互式动画显示（推荐，依次观看）")
    print("2. 简单静态图")
    
    choice = input("请输入选择 (1 或 2): ").strip()
    
    if choice == "1":
        visualizer.interactive_animation()
    elif choice == "2":
        # 创建简单静态图
        visualizer.create_simple_plot()
    else:
        print("无效选择，使用交互式动画显示")
        visualizer.interactive_animation()

if __name__ == "__main__":
    main()