#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int16MultiArray
from sensor_msgs.msg import JointState
from moveit_msgs.msg import RobotState

def servo_callback(msg):
    # Assuming that msg.data contains the servo angles
    joint_state = JointState()
    joint_state.name = ["Revolute2", "Revolute4", "Revolute6", "Revolute9", "Revolute41", "Revolute45"]
    joint_state.position = [angle for angle in msg.data]

    # Assuming that you have a publisher for joint states in MoveIt
    moveit_joint_state_pub.publish(joint_state)

if __name__ == "__main__":
    rospy.init_node("servo_to_moveit_bridge")

    # Set up a subscriber to the /servo topic
    rospy.Subscriber("servo", Int16MultiArray, servo_callback)

    # Set up a publisher for joint states in MoveIt
    moveit_joint_state_pub = rospy.Publisher("/move_group/made_up_node", JointState, queue_size=1)

    rospy.spin()

