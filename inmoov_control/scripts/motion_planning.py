#!/usr/bin/env python

import sys
import rospy
from moveit_commander import RobotCommander, MoveGroupCommander
from moveit_commander import PlanningSceneInterface, roscpp_initialize, roscpp_shutdown
from geometry_msgs.msg import PoseStamped, Pose
from moveit_msgs.msg import Grasp, GripperTranslation, PlaceLocation
from trajectory_msgs.msg import JointTrajectoryPoint

rospy.init_node("motion_planning")

scene = PlanningSceneInterface()
robot = RobotCommander()
L_arm = MoveGroupCommander("L_arm")


rospy.sleep (1)



#print (robot.get_current_variable_values())

print (L_arm.get_current_pose())
print (L_arm.get_planning_frame())
L_arm.set_planning_time(10);
L_arm.set_pose_reference_frame("omo_L")
L_arm.set_position_target([-0.068, -0.120, 0.520])
print (L_arm.get_end_effector_link())
plan = L_arm.plan()
print (plan)
L_arm.execute(plan)
print (L_arm.get_current_pose(L_arm.get_end_effector_link()))
