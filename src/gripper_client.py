#! /usr/bin/env python

import actionlib
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, PoseStamped, Quaternion
from std_msgs.msg import Int32MultiArray
from hsrb_interface import Robot
from hsrb_interface import exceptions
from manip_prelim.msg import *

import rospy

def main():
	client = actionlib.SimpleActionClient('gripperaction', manip_prelim.msg.MoveGripperAction)

	client.wait_for_server()

	curr_time=rospy.get_time()
	while(True):
		if rospy.get_time()-curr_time >= 1:
			goal = manip_prelim.msg.MoveGripperGoal()
			print "requesting"
			client.send_goal(goal)
			#rospy.Duration(5.0)
			client.wait_for_result()
			curr_time = rospy.get_time()

if __name__ == '__main__':
	rospy.init_node('gripper_test_client')
	main()


