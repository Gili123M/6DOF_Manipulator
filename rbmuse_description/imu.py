#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32MultiArray
import serial

def talker():
    pub = rospy.Publisher('imu_data', Float32MultiArray, queue_size=10)
    rospy.init_node('sensor_publisher', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    ser = serial.Serial('/dev/ttyACM0', 115200) # Adjust serial port and baud rate

    while not rospy.is_shutdown():
        data = ser.readline().decode().strip().split(',') # Read data from Arduino
        if len(data) == 3: # Assuming three values (e.g., x, y, z)
            try:
                values = [float(x) for x in data]
                rospy.loginfo(values)
                pub.publish(Float32MultiArray(data=values))
            except ValueError:
                pass

        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

