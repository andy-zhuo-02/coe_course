#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import String
import numpy as np
import time
import sys

class WaypointFlight:
    def __init__(self, vehicle_type='iris', vehicle_id='0'):
        self.vehicle_type = vehicle_type
        self.vehicle_id = vehicle_id
        
        # 初始化ROS节点
        rospy.init_node(f'waypoint_flight_{vehicle_type}_node_{vehicle_id}')
        
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
        
        # 设置目标位置到达阈值
        self.threshold = 0.2
        
        # 预定义航点（示例：以起点为原点的正方形轨迹）
        self.waypoints = [
            [0, 0, 2],    # 起飞点，高度2米
            [2, 0, 2],    # 第一个航点
            [2, 2, 2],    # 第二个航点
            [0, 2, 2],    # 第三个航点
            [0, 0, 2],    # 返回起点
            [0, 0, 0]     # 降落点
        ]
        
    def pose_callback(self, msg):
        """接收当前位置信息的回调函数"""
        self.current_pose = msg.pose
        
    def send_cmd(self, cmd):
        """发送控制指令"""
        self.cmd_pub.publish(String(cmd))
        rospy.sleep(2)  # 等待指令执行
        
    def send_pose(self, position):
        """发送位置指令"""
        pose = Pose()
        pose.position.x = position[0]
        pose.position.y = position[1]
        pose.position.z = position[2]
        pose.orientation.w = 1.0
        self.cmd_pose_pub.publish(pose)
        
    def distance_to_target(self, target):
        """计算当前位置到目标位置的距离"""
        if self.current_pose is None:
            return float('inf')
        current = self.current_pose.position
        return np.sqrt(
            (current.x - target[0])**2 + 
            (current.y - target[1])**2 + 
            (current.z - target[2])**2
        )
        
    def wait_for_arrival(self, target, timeout=30):
        """等待无人机到达目标位置"""
        start_time = time.time()
        rate = rospy.Rate(10)  # 10Hz
        
        while not rospy.is_shutdown():
            if self.distance_to_target(target) < self.threshold:
                rospy.loginfo("到达目标点")
                return True
                
            if time.time() - start_time > timeout:
                rospy.logwarn("到达目标点超时")
                return False
                
            rate.sleep()
            
    def execute_mission(self):
        """执行预定航点任务"""
        rospy.sleep(2)  # 等待连接建立
        
        # 解锁无人机
        self.send_cmd('ARM')
        rospy.sleep(2)
        
        # 切换到OFFBOARD模式
        self.send_cmd('OFFBOARD')
        rospy.sleep(2)
        
        # 按顺序访问每个航点
        for i, waypoint in enumerate(self.waypoints):
            rospy.loginfo(f"前往第 {i+1} 个航点: {waypoint}")
            
            # 发送位置指令
            self.send_pose(waypoint)
            
            # 等待到达目标位置
            if not self.wait_for_arrival(waypoint):
                rospy.logerr("任务执行失败")
                break
                
            rospy.sleep(1)  # 在每个航点停留1秒
            
        # 任务完成后降落
        self.send_cmd('AUTO.LAND')
        rospy.loginfo("任务完成")

if __name__ == '__main__':
    vehicle_type = 'iris'
    vehicle_id = '0'

    wp_flight = WaypointFlight(vehicle_type, vehicle_id)
    wp_flight.execute_mission()
    