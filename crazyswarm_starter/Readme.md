# Crazyswarm 简介

Crazyswarm 是一个用于控制多架 Crazyflie 微型无人机的开源框架。它支持高效的多机编队飞行，提供了强大的工具链和接口，适用于研究和教学场景。

## 功能特点
- 支持多达数十架 Crazyflie 的同步控制
- 提供 Python 和 ROS 接口
- 支持实时轨迹规划和控制
- 提供仿真环境

---

## 安装指南

### 1. 系统要求
- 操作系统：Ubuntu 20.04
- ROS 版本：ROS Noetic

### 2. 安装步骤

具体安装步骤，请参考[Crazyswarm Installation](https://crazyswarm.readthedocs.io/en/latest/installation.html)。

建议跟随Physical Robots and Simulation部分进行安装。

---

## 基本使用指南

### 1. 前期准备
将以下命令添加到`~/.bashrc`文件中：
```
source /YOUR_CRAZYSWARM_DIR/ros_ws/devel/setup.bash
```
其中YOUR_CRAZYSWARM_DIR是你放置你的crazyswarm文件夹的路径。

### 2. 运行示例脚本
打开新的窗口，使用 Python 脚本控制无人机：
```bash
cd ros_ws/src/crazyswarm/scripts
python3 hello_world.py --sim
```


---

## 学习建议
在学习crazyswarm时建议首先将`ros_ws/src/crazyswarm/scripts`中的代码进行尝试和学习，并结合以下链接：
- 官方文档：[Crazyswarm API](https://crazyswarm.readthedocs.io/en/latest/api.html)

