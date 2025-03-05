import rospy
import tf
from geometry_msgs.msg import Quaternion
from tf.transformations import euler_from_quaternion, quaternion_from_euler

#!/usr/bin/env python


def quaternion_to_euler(quat):
    euler = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
    rospy.loginfo("Euler angles: roll=%f, pitch=%f, yaw=%f" % (euler[0], euler[1], euler[2]))
    return euler

def euler_to_quaternion(roll, pitch, yaw):
    quat = quaternion_from_euler(roll, pitch, yaw)
    rospy.loginfo("Quaternion: x=%f, y=%f, z=%f, w=%f" % (quat[0], quat[1], quat[2], quat[3]))
    return Quaternion(*quat)

if __name__ == '__main__':
    rospy.init_node('tf_trans_quat_euler')

    # Example quaternion
    quat = Quaternion(0.0, 0.0, 0.0, 1.0)
    rospy.loginfo("Original Quaternion: x=%f, y=%f, z=%f, w=%f" % (quat.x, quat.y, quat.z, quat.w))

    # Convert quaternion to euler
    euler = quaternion_to_euler(quat)

    # Convert euler back to quaternion
    quat_converted = euler_to_quaternion(euler[0], euler[1], euler[2])
    rospy.loginfo("Converted Quaternion: x=%f, y=%f, z=%f, w=%f" % (quat_converted.x, quat_converted.y, quat_converted.z, quat_converted.w))

    rospy.spin()