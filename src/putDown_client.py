#! /usr/bin/env python

import actionlib
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, PoseStamped, Quaternion
from std_msgs.msg import Int32MultiArray
from hsrb_interface import Robot
from hsrb_interface import exceptions
from hsr_manipulation_2019.msg import *

import rospy


_ORIGIN_TF = 'head_rgbd_sensor_link'

def main():
	client = actionlib.SimpleActionClient('putDownAction', hsr_manipulation_2019.msg.pickUpAction)

	client.wait_for_server()

	curr_time=rospy.get_time()

	goal = hsr_manipulation_2019.msg.putDownGoal()

	# base frame
	# goal.target_pose.pose.position.x=0.5
	# goal.target_pose.pose.position.y=0.0
	# goal.target_pose.pose.position.z=0.4
	# goal.target_pose.pose.orientation.x=0.0
	# goal.target_pose.pose.orientation.y=0.0
	# goal.target_pose.pose.orientation.z=0.0
	# goal.target_pose.pose.orientation.w=0.0
	# goal.target_pose.header.frame_id=_ORIGIN_TF
	
	# rgbd
	goal.target_pose.pose.position.x=0.0
	goal.target_pose.pose.position.y=.4 #.3 #0.5
	goal.target_pose.pose.position.z=1
	goal.target_pose.pose.orientation.x=0.0
	goal.target_pose.pose.orientation.y=0.0
	goal.target_pose.pose.orientation.z=0.0
	goal.target_pose.pose.orientation.w=0.0
	goal.target_pose.header.frame_id=_ORIGIN_TF


	print "requesting"
	client.send_goal(goal)
	#rospy.Duration(5.0)
	client.wait_for_result()
	curr_time = rospy.get_time()

if __name__ == '__main__':
	rospy.init_node('pickup_test_client')
	main()


