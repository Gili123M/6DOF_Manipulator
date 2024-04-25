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
    # Constructor
    def __init__(self):
        rospy.init_node('node_eg3_set_end_effector_orientation', anonymous=True)
        self._planning_group = "group"
        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander()
        self._scene = moveit_commander.PlanningSceneInterface()
        self._group = moveit_commander.MoveGroupCommander(self._planning_group)
        self._display_trajectory_publisher = rospy.Publisher(
            '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)
        self._exectute_trajectory_client = actionlib.SimpleActionClient(
            'execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()
        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()

        rospy.loginfo('\033[94m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
        rospy.loginfo('\033[94m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
        rospy.loginfo('\033[94m' + "Group Names: {}".format(self._group_names) + '\033[0m')
        rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')

    def set_end_effector_orientation(self, desired_orientation):
        pose_target = geometry_msgs.msg.Pose()
        pose_target.orientation = desired_orientation
        self._group.set_pose_target(pose_target)

        # Get the plan
        plan = self._group.plan()

        if isinstance(plan, tuple):
            plan = plan[0]

        if plan:
            # Check if the plan is not empty
            if plan.joint_trajectory.points:
                self._group.execute(plan)
                rospy.loginfo('\033[94m' + ">>> set_end_effector_orientation() Success" + '\033[0m')
                return True
            else:
                rospy.logerr('\033[94m' + ">>> set_end_effector_orientation() Failed: Empty plan." + '\033[0m')
                return False
        else:
            rospy.logerr('\033[94m' + ">>> set_end_effector_orientation() Failed: Planning failed." + '\033[0m')
            return False

    # Destructor
    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo('\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')

def main():
    ur5 = Ur5Moveit()

    # Define desired orientations (quaternion)
    orientation_1 = geometry_msgs.msg.Quaternion()
    orientation_1.x = -0.1469076107565028
    orientation_1.y = 0.0011641319957263958
    orientation_1.z = 5.876486601344748e-06
    orientation_1.w = 0.9891495330150945

    while not rospy.is_shutdown():
        ur5.set_end_effector_orientation(orientation_1)
        rospy.sleep(2)

    del ur5

if __name__ == '__main__':
    main()

