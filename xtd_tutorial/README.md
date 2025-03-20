# XTD Tutorial 使用说明

本教程包含两个示例：单机循迹预定路径点和单机围绕定点盘旋。

## 1. 单机循迹预定路径点

### 启动步骤

1. 启动仿真环境
```bash
roslaunch xtd_tutorial iris_outdoor1.launch
```

2. 启动通信节点
```bash
cd ~/XTDrone/communication/
python multirotor_communication.py iris 0
```

3. 启动循迹控制
```bash
roslaunch xtd_tutorial iris_waypoint_track.launch
```

## 2. 单机围绕定点盘旋

### 启动步骤

1. 启动仿真环境
```bash
roslaunch xtd_tutorial iris_circular.launch
```

2. 启动通信节点
```bash
cd ~/XTDrone/communication/
python multirotor_communication.py iris 0
```

3. 启动盘旋控制
```bash
roslaunch xtd_tutorial iris_circular_track.launch
```

## 通信节点接口文档 (multirotor_communication.py)

通信节点负责将高级控制命令转换为无人机可以理解的低级控制指令。

### 使用方法

```bash
python multirotor_communication.py <vehicle_type> <vehicle_id>
```

### 订阅主题 (Subscribers)

| 主题名称 | 消息类型 | 说明 |
|---------|---------|------|
| `{vehicle_type}_{vehicle_id}/mavros/local_position/pose` | PoseStamped | 获取当前位置信息 |
| `/xtdrone/{vehicle_type}_{vehicle_id}/cmd` | String | 接收控制命令 |
| `/xtdrone/{vehicle_type}_{vehicle_id}/cmd_pose_flu` | Pose | 接收前左上(FLU)坐标系下的位置命令 |
| `/xtdrone/{vehicle_type}_{vehicle_id}/cmd_pose_enu` | Pose | 接收东北上(ENU)坐标系下的位置命令 |
| `/xtdrone/{vehicle_type}_{vehicle_id}/cmd_vel_flu` | Twist | 接收前左上(FLU)坐标系下的速度命令 |
| `/xtdrone/{vehicle_type}_{vehicle_id}/cmd_vel_enu` | Twist | 接收东北上(ENU)坐标系下的速度命令 |
| `/xtdrone/{vehicle_type}_{vehicle_id}/cmd_accel_flu` | Twist | 接收前左上(FLU)坐标系下的加速度命令 |
| `/xtdrone/{vehicle_type}_{vehicle_id}/cmd_accel_enu` | Twist | 接收东北上(ENU)坐标系下的加速度命令 |

### 发布主题 (Publishers)

| 主题名称 | 消息类型 | 说明 |
|---------|---------|------|
| `{vehicle_type}_{vehicle_id}/mavros/setpoint_raw/local` | PositionTarget | 发布目标运动控制指令 |

### 支持的控制命令

| 命令 | 说明 |
|-----|------|
| ARM | 解锁无人机 |
| DISARM | 上锁无人机 |
| HOVER | 当前位置悬停 |
| OFFBOARD | 切换到外部控制模式 |
| AUTO.LAND | 自动降落 |

### 坐标系说明

- **ENU坐标系**: 东北天坐标系(East-North-Up)，是一种常用的全局坐标系
- **FLU坐标系**: 前左上坐标系(Forward-Left-Up)，以无人机机体为参考的局部坐标系

## 注意事项

- 请确保按顺序执行启动步骤
- 每个命令执行后，请等待其完全启动再进行下一步
- 如需停止仿真，请使用 Ctrl+C 安全退出

