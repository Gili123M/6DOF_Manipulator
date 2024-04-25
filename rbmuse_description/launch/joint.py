#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
import serial
import time

# Serial communication settings
serial_port = '/dev/ttyACM0'  # Replace with your Arduino's port
baud_rate = 115200
ser = serial.Serial(serial_port, baud_rate, timeout=1)

# Callback function for joint state messages
def joint_states_callback(msg):
    joint_positions = msg.position  # Get joint positions from the message
    
    # Convert joint positions to a string with comma separation
    joint_positions_str = ','.join(map(str, joint_positions))
    
    # Send joint positions to Arduino via serial communication
    ser.write(joint_positions_str.encode())
    rospy.loginfo("Sending radian values of each joint to Arduino: %s", joint_positions_str)

def joint_angles_node():
    rospy.init_node('joint_angles_node', anonymous=True)
    rospy.Subscriber('/joint_states', JointState, joint_states_callback)
    
    # Main loop to keep the node running
    rate = rospy.Rate(10)  # Adjust the rate as needed
    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    try:
        joint_angles_node()
    except rospy.ROSInterruptException:
        pass
