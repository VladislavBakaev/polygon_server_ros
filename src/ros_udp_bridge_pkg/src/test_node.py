#!/usr/bin/env python3
import rospy  
from inverse_problem_srv.srv import point_cmd,point_cmdResponse
from std_srvs.srv import SetBool

srv_ang_point = rospy.ServiceProxy('/angle_robot/cmd_point',point_cmd)
srv_pal_point = rospy.ServiceProxy('/palletizer_robot/cmd_point',point_cmd)

srv_ang_gripper = rospy.ServiceProxy('/angle_robot/gripper_cmd',SetBool)
srv_pal_vacuum = rospy.ServiceProxy('/palletizer_robot/vacuum',SetBool)

srv_ang_red_light = rospy.ServiceProxy('/angle_robot/red_light',SetBool)
srv_ang_green_light = rospy.ServiceProxy('/angle_robot/green_light',SetBool)

srv_pal_red_light = rospy.ServiceProxy('/palletizer_robot/red_light',SetBool)
srv_pal_green_light = rospy.ServiceProxy('/palletizer_robot/green_light',SetBool)
print(1)
srv_ang_red_light(False)
print(2)
srv_ang_green_light(True)
print(3)
srv_ang_point('150 150 50 1')
print(4)
srv_ang_gripper(False)
print(5)
srv_ang_point('150 150 150 -1')
print(6)
srv_ang_red_light(True)
print(7)
srv_ang_green_light(False)
print(8)
srv_ang_gripper(True)
print(9)

srv_pal_red_light(False)
print(10)
srv_pal_green_light(True)
print(11)
srv_pal_point('150 150 100')
print(12)
srv_pal_vacuum(True)
print(13)
srv_pal_point('250 250 150')
print(14)
srv_pal_red_light(True)
print(15)
srv_pal_green_light(False)
print(16)
srv_pal_vacuum(False)
print(17)


