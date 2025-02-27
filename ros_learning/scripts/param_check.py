import rospy

if __name__ == '__main__':
    rospy.init_node('param_check_node', anonymous=True)
    robot_name = rospy.get_param('/robot_info/name')
    max_speed = rospy.get_param('/max_speed')
    msg = rospy.get_param('/param_check_node/input_message')

    print(f"Robot Name: {robot_name}")
    print(f"Max speed: {max_speed}")
    print(f"Input message: {msg}")
