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

## 注意事项

- 请确保按顺序执行启动步骤
- 每个命令执行后，请等待其完全启动再进行下一步
- 如需停止仿真，请使用 Ctrl+C 安全退出
