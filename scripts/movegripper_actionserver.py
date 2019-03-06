#! /usr/bin/env python
import rospy
import actionlib

from std_msgs.msg import *
from geometry_msgs.msg import *
from hsrb_interface import Robot, exceptions, geometry

from manip_prelim.msg import *
import math

class MoveGripperAction(object):

	def __init__(self, name, robot):
		self.action_name = name
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

		self._as = actionlib.SimpleActionServer(self.action_name, manip_prelim.msg.MoveGripperAction, execute_cb=self.execute_cb, auto_start=False)
		self._as.start()

	def execute_cb(self, goal):
		# self.body.move_to_neutral()
		# rospy.sleep(3)
		self.body.move_to_joint_positions({'arm_flex_joint': -1.57, 'arm_lift_joint':0.1})
		if self.gripper_state==True:
			self.close_gripper()
			self.gripper_state=False
			rospy.loginfo("close_gripper")
			self.body.move_to_joint_positions({'arm_lift_joint':0.65})
		else:
			self.open_gripper()
			self.gripper_state=True
			rospy.loginfo("open_gripper")
			self.body.move_to_neutral()


		
		# rospy.sleep(3)	
		# self.open_gripper()
		# rospy.loginfo("open_gripper")
		# rospy.sleep(3)
		# self.close_gripper()
		# rospy.loginfo("close_gripper")
		self._as.set_succeeded()

	def open_gripper(self,to_width=1.2):
		self.gripper.command(to_width)

	def close_gripper(self, to_width=-0.01):
		# self.gripper.grasp(to_width)
		self.gripper.apply_force(1.0)

if __name__ == '__main__':
	robot = Robot()
	rospy.loginfo("Initializing givepose server")
	server=MoveGripperAction('gripperaction',robot)
	rospy.loginfo("movegripper_action_server created")
	rospy.spin()

