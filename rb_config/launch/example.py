#!/usr/bin/env python3
import rospy
import actionlib
import control_msgs.msg as control_msgs
import trajectory_msgs.msg as trajectory_msgs

def move_to_pose(pose):
    goal = control_msgs.FollowJointTrajectoryGoal()
    # Fill in the details of the goal message with the desired pose
    # For example, if you are controlling a robotic arm, you would specify joint positions here.
    # If you are controlling a mobile base, you would specify velocity commands here.
    # This example assumes you are controlling joint positions.
    goal.trajectory.joint_names = ["Revolute2", "Revolute4", "Revolute6", "Revolute9", "Revolute41", "Revolute43"]
    point = trajectory_msgs.JointTrajectoryPoint()  # Corrected attribute name
    point.positions = pose
    point.time_from_start = rospy.Duration(2.0)  # Adjust as needed
    goal.trajectory.points.append(point)

    client.send_goal(goal)
    client.wait_for_result()

if __name__ == "__main__":
    rospy.init_node("move_to_poses")

    client = actionlib.SimpleActionClient("follow_joint_trajectory", control_msgs.FollowJointTrajectoryAction)
    client.wait_for_server()

    poses = [
        [0.1, 0.2, 0.3, 0.4, 0.5, 0.6],  # Example pose 1
        [0.2, 0.3, 0.4, 0.5, 0.6, 0.7],  # Example pose 2
        # Add more poses as needed
    ]

    for pose in poses:
        move_to_pose(pose)

    rospy.loginfo("All poses executed successfully.")

