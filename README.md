# 6DOF_Manipulator
- A Basic 6 DOF DIY Manipulator which can be controlled through ROS 
- The rbmuse_description is the file that is generated directly from the FUsion 360 by the using the plugin 'fusion2urdf'
- The rb_config file is the configuration file made through MoveIt Assistant Setup the URDF file is uploaded in the MoveIt Assistant Setup
- The Servo_Motor_Code is the Arduino code that accepts the radian values from the MoveIt and convert it into angular values and pass it to the Servo Motor
- To connect the Arduino with the ROS you need to use few commands, you need to open multiple terminals.
   - Terminal 1 "_roscore_"
   - Terminal 2 "_sudo chmod a+rw /dev/ttyACM0_" This gives the permission to the specific USB channel that is connected to the Arduino "ttyACM0" can wary
   - Terminal 3 "_rosrun rosserial_python serila_node.py _port:=/dev/ttyACM0 _baud:= 115200_" this is the command which helps in sending the information from ROS to the Arduino
   - Terminal 4 either "_roslaunch rb_congfig demo.launch_" or "_roslaunch rbmuse_description display.launch_" the rb_config launch file already has predifned poses configures in the MoveIt.
     The rbmuse launch file in that you can independently control the servo motors.
   - Terminal 5 "_rostopic echo /joint_states_" this topic present in the MoveIt which shows the values of the Servo motors in Radian values.
   - Terminal 6 "_rostopic echo /servo_" first you need to upload the arduino code in the arduino board then only this topic will be initialised where it shows the angular values recieved from the 
     joint_states
