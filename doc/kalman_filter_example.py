#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
卡尔曼滤波器示例：车辆在一维直线上的位置估计
此脚本演示了如何使用卡尔曼滤波器结合IMU加速度数据和低频全局位置数据
来估计一个车辆在一维直线上的位置。
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import matplotlib as mpl
from matplotlib.font_manager import FontProperties
import time

# 配置中文字体
# 方法1：使用系统中已知的中文字体
font_path = '/usr/share/fonts/truetype/arphic/uming.ttc'  # AR PL UMing
# 备选字体
# font_path = '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'  # Noto Sans CJK

# 创建字体属性对象
chinese_font = FontProperties(fname=font_path)

# 设置全局字体参数
plt.rcParams['axes.unicode_minus'] = False  # 用来正常显示负号

# 设置随机种子以保证结果可重现
np.random.seed(42)

class KalmanFilter:
    def __init__(self, dt):
        """
        初始化卡尔曼滤波器
        
        参数:
        dt -- 时间步长
        """
        # 状态向量: [位置, 速度, 加速度]
        self.x = np.zeros(3)
        
        # 状态转移矩阵
        self.F = np.array([
            [1, dt, 0.5*dt*dt],
            [0, 1, dt],
            [0, 0, 1]
        ])
        
        # 观测矩阵 (位置观测)
        self.H_pos = np.array([[1, 0, 0]])
        
        # 观测矩阵 (加速度观测)
        self.H_acc = np.array([[0, 0, 1]])
        
        # 状态协方差矩阵
        self.P = np.eye(3) * 1000  # 初始不确定性较大
        
        # 过程噪声协方差矩阵
        self.Q = np.array([
            [0.01, 0, 0],
            [0, 0.01, 0],
            [0, 0, 0.1]
        ])
        
        # 测量噪声协方差 (位置)
        self.R_pos = np.array([[10.0]])
        
        # 测量噪声协方差 (加速度)
        self.R_acc = np.array([[1.0]])
    
    def predict(self):
        """预测步骤"""
        # 更新状态
        self.x = self.F @ self.x
        
        # 更新协方差
        self.P = self.F @ self.P @ self.F.T + self.Q
        
        return self.x[0]  # 返回位置估计
    
    def update_with_position(self, z_pos):
        """
        使用位置观测进行更新
        
        参数:
        z_pos -- 位置观测值
        """
        # 计算卡尔曼增益
        K = self.P @ self.H_pos.T @ np.linalg.inv(self.H_pos @ self.P @ self.H_pos.T + self.R_pos)
        
        # 更新状态
        self.x = self.x + K @ (z_pos - self.H_pos @ self.x)
        
        # 更新协方差
        self.P = (np.eye(3) - K @ self.H_pos) @ self.P
        
        return self.x[0]  # 返回位置估计
    
    def update_with_acceleration(self, z_acc):
        """
        使用加速度观测进行更新
        
        参数:
        z_acc -- 加速度观测值
        """
        # 计算卡尔曼增益
        K = self.P @ self.H_acc.T @ np.linalg.inv(self.H_acc @ self.P @ self.H_acc.T + self.R_acc)
        
        # 更新状态
        self.x = self.x + K @ (z_acc - self.H_acc @ self.x)
        
        # 更新协方差
        self.P = (np.eye(3) - K @ self.H_acc) @ self.P
        
        return self.x[0]  # 返回位置估计

# 仿真参数
simulation_time = 100  # 仿真总时间
dt = 0.1  # 时间步长
time_steps = int(simulation_time / dt)
time = np.linspace(0, simulation_time, time_steps)

# 真实状态
true_acc = np.zeros(time_steps)
true_vel = np.zeros(time_steps)
true_pos = np.zeros(time_steps)

# 生成真实加速度（随机变化）
for i in range(1, time_steps):
    if i % 100 == 0:
        true_acc[i:] = np.random.normal(0, 0.5)
    
    # 通过积分计算真实速度和位置
    true_vel[i] = true_vel[i-1] + true_acc[i] * dt
    true_pos[i] = true_pos[i-1] + true_vel[i] * dt + 0.5 * true_acc[i] * dt**2

# 生成带噪声的IMU加速度测量值 (高频率, 每个时间步)
imu_acc = true_acc + np.random.normal(0, 1, time_steps)

# 生成带噪声的全局位置测量值 (低频率, 每10个时间步)
global_pos_indices = np.arange(0, time_steps, 10)
global_pos = true_pos[global_pos_indices] + np.random.normal(0, 2, len(global_pos_indices))
global_pos_times = time[global_pos_indices]

# 初始化卡尔曼滤波器
kf = KalmanFilter(dt)

# 存储卡尔曼滤波器估计结果
estimated_pos = np.zeros(time_steps)

# 里程计估计（仅通过IMU积分）
odom_vel = np.zeros(time_steps)
odom_pos = np.zeros(time_steps)

# 运行卡尔曼滤波器
next_global_idx = 0
for i in range(time_steps):
    # 预测步骤
    estimated_pos[i] = kf.predict()
    
    # 使用IMU加速度测量值更新
    kf.update_with_acceleration(imu_acc[i])
    
    # 当有全局位置测量值时，使用它进行更新
    if next_global_idx < len(global_pos_indices) and i == global_pos_indices[next_global_idx]:
        kf.update_with_position(global_pos[next_global_idx])
        next_global_idx += 1
    
    # 更新里程计估计
    if i > 0:
        odom_vel[i] = odom_vel[i-1] + imu_acc[i] * dt
        odom_pos[i] = odom_pos[i-1] + odom_vel[i] * dt + 0.5 * imu_acc[i] * dt**2

# 创建交互式绘图
fig, ax = plt.subplots(figsize=(12, 6))
animation_finished = False  # 标记动画是否已完成

def init():
    ax.clear()
    ax.set_xlim(0, simulation_time)
    ymin = min(np.min(true_pos), np.min(estimated_pos), np.min(odom_pos), np.min(global_pos)) - 5
    ymax = max(np.max(true_pos), np.max(estimated_pos), np.max(odom_pos), np.max(global_pos)) + 5
    ax.set_ylim(ymin, ymax)
    ax.set_xlabel('时间 (秒)', fontproperties=chinese_font)
    ax.set_ylabel('位置', fontproperties=chinese_font)
    ax.set_title('卡尔曼滤波器位置估计 vs 真实位置', fontproperties=chinese_font)
    ax.grid(True)
    return []

def update(frame):
    global animation_finished
    
    # 如果已经完成，则不再更新
    if animation_finished:
        return []
    
    ax.clear()
    
    # 显示更多时间帧
    end_idx = min(frame + 1, time_steps)
    
    # 绘制真实位置
    ax.plot(time[:end_idx], true_pos[:end_idx], 'g-', label='真实位置')
    
    # 绘制卡尔曼滤波器估计位置
    ax.plot(time[:end_idx], estimated_pos[:end_idx], 'r--', label='卡尔曼滤波估计')
    
    # 绘制里程计估计位置
    ax.plot(time[:end_idx], odom_pos[:end_idx], 'b:', label='里程计估计')
    
    # 绘制全局位置测量
    global_indices_to_show = global_pos_indices[global_pos_indices < end_idx]
    if len(global_indices_to_show) > 0:
        ax.scatter(time[global_indices_to_show], global_pos[:len(global_indices_to_show)], 
                  color='purple', marker='x', s=50, label='全局位置测量')
    
    ax.set_xlim(0, simulation_time)
    ymin = min(np.min(true_pos), np.min(estimated_pos), np.min(odom_pos), np.min(global_pos)) - 5
    ymax = max(np.max(true_pos), np.max(estimated_pos), np.max(odom_pos), np.max(global_pos)) + 5
    ax.set_ylim(ymin, ymax)
    ax.set_xlabel('时间 (秒)', fontproperties=chinese_font)
    ax.set_ylabel('位置', fontproperties=chinese_font)
    ax.set_title('卡尔曼滤波器位置估计 vs 真实位置', fontproperties=chinese_font)
    ax.grid(True)
    # 使用中文字体显示图例
    legend = ax.legend(loc='upper left', prop=chinese_font)
    
    # 检查是否是最后一帧，如果是则保存静态图像并关闭动画窗口
    if frame == time_steps - 1:
        animation_finished = True
        plt.savefig('kalman_filter_animation_last_frame.png')
        print("动画最后一帧已保存为 'kalman_filter_animation_last_frame.png'")
        
        # 使用Timer安排在主循环中执行清理和关闭操作
        timer = fig.canvas.new_timer(interval=500)  # 500毫秒后执行
        timer.add_callback(finish_animation)
        timer.start()
    
    return []

def finish_animation():
    """完成动画后的清理工作"""
    save_final_plot()      # 生成完整静态图像
    plt.close(fig)         # 关闭动画窗口
    
def save_final_plot():
    """生成并保存完整静态图像"""
    print("正在生成完整结果图像...")
    static_fig = plt.figure(figsize=(12, 6))
    plt.plot(time, true_pos, 'g-', label='真实位置')
    plt.plot(time, estimated_pos, 'r--', label='卡尔曼滤波估计')
    plt.plot(time, odom_pos, 'b:', label='里程计估计')
    plt.scatter(global_pos_times, global_pos, color='purple', marker='x', s=50, label='全局位置测量')
    plt.xlabel('时间 (秒)', fontproperties=chinese_font)
    plt.ylabel('位置', fontproperties=chinese_font)
    plt.title('卡尔曼滤波器位置估计对比', fontproperties=chinese_font)
    plt.grid(True)
    plt.legend(prop=chinese_font)
    plt.tight_layout()
    plt.savefig('kalman_filter_results.png')
    print("完整结果图已保存为 'kalman_filter_results.png'")
    plt.close(static_fig)  # 关闭静态图像窗口
    print("脚本执行完毕")

# 运行动画
if __name__ == "__main__":
    print("卡尔曼滤波器示例：一维直线上车辆的位置估计")
    print("此脚本将显示一个动画，展示卡尔曼滤波器如何结合IMU加速度和低频全局位置")
    print("绿线: 真实位置")
    print("红线: 卡尔曼滤波估计")
    print("蓝线: 基于IMU的里程计估计")
    print("紫色x: 低频率全局位置测量")
    print("动画将自动播放完成后生成并保存图像")
    
    # 检查字体是否存在
    import os
    if os.path.exists(font_path):
        print(f"使用中文字体: {font_path}")
    else:
        print(f"警告: 字体文件 {font_path} 不存在")
        print("请修改字体路径或安装相应的字体")

    # 创建动画，设置repeat=False确保动画只播放一次
    ani = FuncAnimation(fig, update, frames=time_steps, init_func=init, 
                       blit=False, interval=50, repeat=False)
    # 显示动画
    plt.tight_layout()
    plt.show() 