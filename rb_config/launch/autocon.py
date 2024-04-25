#!/usr/bin/env python3

import rospy
import sys
import moveit_commander
import moveit_msgs.msg
import actionlib
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import serial  # Add this import

class Ur5Moveit:
    def __init__(self, arduino_port):
        try:
            rospy.init_node('node_eg2_predefined_pose', anonymous=True)
            self._planning_group = "group"
            self._commander = moveit_commander.roscpp_initialize(sys.argv)
            self._robot = moveit_commander.RobotCommander()
            self._scene = moveit_commander.PlanningSceneInterface()
            self._group = moveit_commander.MoveGroupCommander(self._planning_group)
            self._execute_trajectory_client = actionlib.SimpleActionClient(
                'execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
            self._execute_trajectory_client.wait_for_server()

            self._arduino = serial.Serial(arduino_port, 9600, timeout=1)  # Adjust the baud rate accordingly

        except Exception as e:
            rospy.logerr("Error during initialization: {}".format(str(e)))
            raise  # Re-raise the exception to terminate the program

    def go_to_predefined_pose(self, arg_pose_name):
        rospy.loginfo('\033[94m' + "Going to Pose: {}".format(arg_pose_name) + '\033[0m')
        self._group.set_named_target(arg_pose_name)

        # Plan and get the result
        plan_result = self._group.plan()

        # Check if planning was successful
        if plan_result[0]:
            plan = plan_result[1].joint_trajectory
            # Extract joint positions from the first point in the trajectory
            joint_positions = plan.points[0].positions

            # Create a trajectory message
            traj_msg = moveit_msgs.msg.RobotTrajectory()
            traj_msg.joint_trajectory = JointTrajectory()
            traj_msg.joint_trajectory.joint_names = self._group.get_active_joints()

            # Create a JointTrajectoryPoint and append it
            joint_trajectory_point = JointTrajectoryPoint(positions=joint_positions)
            traj_msg.joint_trajectory.points.append(joint_trajectory_point)

            goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
            goal.trajectory = traj_msg

            self._execute_trajectory_client.send_goal(goal)
            self._execute_trajectory_client.wait_for_result()

            # Send a signal to Arduino after each pose
            self._arduino.write(b'1')  # Sending '1' to Arduino

            rospy.loginfo('\033[92m' + "Now at Pose: {}".format(arg_pose_name) + '\033[0m')
        else:
            rospy.logwarn('\033[91m' + "Failed to plan for pose: {}".format(arg_pose_name) + '\033[0m')

    def __del__(self):
        if hasattr(self, '_arduino'):
            self._arduino.close()  # Close the serial connection
            rospy.loginfo('\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')

def main():
    arduino_port = "/dev/ttyACM0"  # Replace with the correct port for your Arduino
    ur5 = Ur5Moveit(arduino_port)
    try:
        while not rospy.is_shutdown():
            ur5.go_to_predefined_pose("home")
            rospy.sleep(2)
            ur5.go_to_predefined_pose("pick")
            rospy.sleep(2)
            ur5.go_to_predefined_pose("grip_pose")
            rospy.sleep(2)
            ur5.go_to_predefined_pose("place_pose")
            rospy.sleep(2)
            ur5.go_to_predefined_pose("drop_pose")
            rospy.sleep(2)

    except rospy.ROSInterruptException:
        pass

    del ur5

if __name__ == '__main__':
    main()

