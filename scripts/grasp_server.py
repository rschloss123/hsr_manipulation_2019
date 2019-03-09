#! /usr/bin/env python
import rospy
import actionlib

from std_msgs.msg import *
from geometry_msgs.msg import *
from hsrb_interface import Robot, exceptions, geometry
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

from manip_prelim.msg import *
import math

class GraspAction(object):

	def __init__(self, robot):

		self.robot = robot 
		self.gripper_state=True

		while not rospy.is_shutdown():
			try:
				self.body=self.robot.try_get('whole_body')
				self.gripper = self.robot.try_get('gripper')
				self.open_gripper()
				self.body.move_to_neutral()
				break 
			except(exceptions.ResourceNotFoundError, exceptions.RobotConnectionError) as e:
				rospy.logerr("Failed to obtain resource: {}\nRetrying...".format(e))

		# navigation client to move the base 
		self.navi_cli = actionlib.SimpleActionClient('/move_base/move', MoveBaseAction)

		self._as = actionlib.SimpleActionServer('pickupaction', manip_prelim.msg.GraspAction, execute_cb=self.pickUp, auto_start=False)
		self._as.start()

	def pickUp(self, goal):


		# make sure gripper is open
		self.open_gripper()
		self.gripper_state=True
		rospy.loginfo("open_gripper")

		# calculate transform for arm

		obj_arm_lift_link = 0.2
		# self.body.move_to_joint_positions({'arm_lift_joint':obj_arm_lift_link})

		self.body.move_to_joint_positions({'arm_flex_joint': -1.57, 'arm_lift_joint':obj_arm_lift_link})

		# compute how much to move base 
		# to-do
		goal_x = -0.0
		goal_y = -0.0
		goal_yaw = 0.0

		self.navigation_action(goal_x,goal_y,goal_yaw)  

		self.close_gripper()
		self.gripper_state=False
		rospy.loginfo("close_gripper")

		self._as.set_succeeded()

	def open_gripper(self,to_width=1.2):
		self.gripper.command(to_width)

	def close_gripper(self, to_width=-0.01):
		# self.gripper.grasp(to_width)
		self.gripper.apply_force(1.0)

	def navigation_action(goal_x,goal_y,goal_yaw):
		pose = PoseStamped()
		pose.header.stamp = rospy.Time.now()
		pose.header.frame_id = "map"
		pose.pose.position = Point(goal_x, goal_y, 0)
		quat = tf.transformations.quaternion_from_euler(0, 0, goal_yaw)
		pose.pose.orientation = Quaternion(*quat)

		goal = MoveBaseGoal()
		goal.target_pose = pose

		# send message to the action server
		self.navi_cli.send_goal(goal)

		# wait for the action server to complete the order
		self.navi_cli.wait_for_result()

		# print result of navigation
		result_action_state = self.navi_cli.get_state()

		return #result_action_state 

if __name__ == '__main__':
	robot = Robot()
	rospy.loginfo("Initializing givepose server")
	server=GraspAction(robot)
	rospy.loginfo("grasp_action_server created")
	rospy.spin()

