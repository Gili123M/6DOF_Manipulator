# 6DOF_Manipulator
- A Basic 6 DOF DIY Manipulator which can be controlled through ROS 
- The rbmuse_description is the file that is generated directly from the FUsion 360 by the using the plugin 'fusion2urdf'
- The rb_config file is the configuration file made through MoveIt Assistant Setup the URDF file is uploaded in the MoveIt Assistant Setup
- The Servo_Motor_Code is the Arduino code that accepts the radian values from the MoveIt and convert it into angular values and pass it to the Servo Motor
- To connect the Arduino with the ROS you need to use few commands, you need to open multiple terminals.
   - Termina 1 "_roscore_"
