#!/usr/bin/env python3

import rospy
import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import math

class Ur5Moveit:
    def __init__(self):
        rospy.init_node('node_eg3_set_joint_angles', anonymous=True)
        self._planning_group = "group"
        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander()
        self._scene = moveit_commander.PlanningSceneInterface()
        self._group = moveit_commander.MoveGroupCommander(self._planning_group)
        self._display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)
        self._execute_trajectory_client = actionlib.SimpleActionClient('execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._execute_trajectory_client.wait_for_server()
        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()
        rospy.loginfo('\033[94m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
        rospy.loginfo('\033[94m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
        rospy.loginfo('\033[94m' + "Group Names: {}".format(self._group_names) + '\033[0m')
        rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')

    def set_joint_angles(self, arg_list_joint_angles):
        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Current Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Current Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)
        
        self._group.set_joint_value_target(arg_list_joint_angles)
        self._group.plan()
        flag_plan = self._group.go(wait=True)

        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        if flag_plan:
            rospy.loginfo('\033[94m' + ">>> set_joint_angles() Success" + '\033[0m')
        else:
            rospy.logerr('\033[94m' + ">>> set_joint_angles() Failed." + '\033[0m')

        return flag_plan

    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo('\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')

def main():
    ur5 = Ur5Moveit()
    
    lst_joint_angles_1 = [math.radians(0), math.radians(0), math.radians(0), math.radians(0), math.radians(0), math.radians(-40.107)]
    lst_joint_angles_2 = [math.radians(0), math.radians(-33.231), math.radians(24.63719), math.radians(0), math.radians(0), math.radians(-40.107)]
    lst_joint_angles_3 = [math.radians(40.107), math.radians(-33.231), math.radians(24.63719), math.radians(0), math.radians(0), math.radians(-40.107)]
    lst_joint_angles_4 = [math.radians(127.1966), math.radians(0), math.radians(11.4591), math.radians(0), math.radians(0), math.radians(-49.8475)]
    lst_joint_angles_5 = [math.radians(127.1966), math.radians(0), math.radians(11.4591), math.radians(0), math.radians(0), math.radians(0)]

    while not rospy.is_shutdown():
        ur5.set_joint_angles(lst_joint_angles_1)
        rospy.sleep(2)
        ur5.set_joint_angles(lst_joint_angles_2)
        rospy.sleep(2)
        ur5.set_joint_angles(lst_joint_angles_3)
        rospy.sleep(2)
        ur5.set_joint_angles(lst_joint_angles_4)
        rospy.sleep(2)
        ur5.set_joint_angles(lst_joint_angles_5)
        rospy.sleep(2)
        ur5.set_joint_angles(lst_joint_angles_1)
        rospy.sleep(2)

    del ur5

if __name__ == '__main__':
    main()

