#!/usr/bin/env python
import rospy
import tf2_ros
import geometry_msgs.msg
import tf.transformations 
import math

def print_turtle2_in_turtle1():
    rospy.init_node('turtle2_in_turtle1_listener')
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    rate = rospy.Rate(10.0) # 10 Hz

    while not rospy.is_shutdown():
        try:
            # 获取从turtle1到turtle2的变换
            trans = tf_buffer.lookup_transform('turtle1', 'turtle2', rospy.Time())

            # 提取平移分量
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            z = trans.transform.translation.z

            # 提取四元数并转换为欧拉角
            quat = [
            trans.transform.rotation.x,
            trans.transform.rotation.y,
            trans.transform.rotation.z,
            trans.transform.rotation.w
            ]
            roll, pitch, yaw = tf.transformations.euler_from_quaternion(quat) # 使用tf.transformations模块

            # 打印位置和偏航角
            rospy.loginfo("Turtle2 in Turtle1 frame:")
            rospy.loginfo("Position: x=%.2f, y=%.2f, z=%.2f", x, y, z)
            rospy.loginfo("Yaw: %.2f degrees", math.degrees(yaw))

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn("Exception occurred: %s", e)

        rate.sleep()

if __name__ == "__main__":
    print_turtle2_in_turtle1()