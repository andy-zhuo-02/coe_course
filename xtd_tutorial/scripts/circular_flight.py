#!/usr/bin/env python

import rospy
import math
import numpy as np
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import String
import time

class CircularFlight:
    def __init__(self, vehicle_type='iris', vehicle_id='0'):
        self.vehicle_type = vehicle_type
        self.vehicle_id = vehicle_id
        
        # 初始化ROS节点
        rospy.init_node(f'circular_flight_{vehicle_type}{vehicle_id}')
        
        # 订阅当前位置信息
        self.local_pose_sub = rospy.Subscriber(
            f"{vehicle_type}_{vehicle_id}/mavros/local_position/pose", 
            PoseStamped, 
            self.pose_callback
        )
        
        # 发布位置指令和控制指令
        self.cmd_pose_pub = rospy.Publisher(
            f'/xtdrone/{vehicle_type}_{vehicle_id}/cmd_pose_enu',
            Pose,
            queue_size=1
        )
        self.cmd_pub = rospy.Publisher(
            f'/xtdrone/{vehicle_type}_{vehicle_id}/cmd',
            String,
            queue_size=1
        )
        
        # 初始化当前位置
        self.current_pose = None
        
        # 圆形轨迹参数
        self.center = [-3, 0, 2]  # 圆心坐标 [x, y, z]
        self.radius = 2.0        # 圆形轨迹半径（米）
        self.angular_speed = 0.5 # 角速度（弧度/秒）
        self.current_angle = 0   # 当前角度
        
        # 设置目标位置到达阈值
        self.threshold = 0.2
        
    def pose_callback(self, msg):
        """接收当前位置信息的回调函数"""
        self.current_pose = msg.pose
        
    def send_cmd(self, cmd):
        """发送控制指令"""
        self.cmd_pub.publish(String(cmd))
        rospy.sleep(2)  # 等待指令执行
        
    def calculate_target_pose(self):
        """计算目标位置和朝向"""
        # 计算目标位置
        target_x = self.center[0] + self.radius * math.cos(self.current_angle)
        target_y = self.center[1] + self.radius * math.sin(self.current_angle)
        target_z = self.center[2]
        
        # 计算朝向圆心的四元数
        # 计算从当前位置指向圆心的向量
        dx = self.center[0] - target_x
        dy = self.center[1] - target_y
        yaw = math.atan2(dy, dx)
        
        # 将偏航角转换为四元数
        qw = math.cos(yaw/2)
        qz = math.sin(yaw/2)
        
        return [target_x, target_y, target_z, qw, 0, 0, qz]
        
    def send_pose(self, pose_data):
        """发送位置和姿态指令"""
        pose = Pose()
        pose.position.x = pose_data[0]
        pose.position.y = pose_data[1]
        pose.position.z = pose_data[2]
        pose.orientation.w = pose_data[3]
        pose.orientation.x = pose_data[4]
        pose.orientation.y = pose_data[5]
        pose.orientation.z = pose_data[6]
        self.cmd_pose_pub.publish(pose)
        
    def execute_circular_flight(self):
        """执行圆形轨迹飞行"""
        rospy.sleep(5)  # 等待连接建立
        
        # 解锁无人机
        rospy.loginfo("解锁无人机") 
        self.send_cmd('ARM')
        rospy.sleep(5)
        
        # 切换到OFFBOARD模式
        rospy.loginfo("切换到OFFBOARD模式") 
        self.send_cmd('OFFBOARD')
        rospy.sleep(2)
        
        # 设置循环频率
        rate = rospy.Rate(20)  # 20Hz
        
        rospy.loginfo("开始圆形轨迹飞行")
        
        try:
            while not rospy.is_shutdown():
                # 计算目标位置和姿态
                target_pose = self.calculate_target_pose()
                
                # 发送位置和姿态指令
                self.send_pose(target_pose)
                
                # 更新角度
                self.current_angle += self.angular_speed / 20.0  # 根据频率调整角度增量
                if self.current_angle >= 2 * math.pi:
                    self.current_angle -= 2 * math.pi
                
                rate.sleep()
                
        except KeyboardInterrupt:
            rospy.loginfo("停止圆形轨迹飞行")
            # 降落
            self.send_cmd('AUTO.LAND')
            rospy.loginfo("开始降落")

if __name__ == '__main__':
    try:
        # 创建并执行圆形轨迹飞行任务
        circular_flight = CircularFlight()
        circular_flight.execute_circular_flight()
    except rospy.ROSInterruptException:
        pass 