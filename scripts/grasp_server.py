#! /usr/bin/env python
import rospy
import actionlib
import tf
import tf2_ros
from tf import TransformListener
from std_msgs.msg import *
from geometry_msgs.msg import *
from hsrb_interface import Robot, exceptions, geometry
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sensor_msgs.msg import JointState

from hsr_manipulation_2019.msg import *
import math

import numpy as np 
from numpy import linalg as LA
from numpy.linalg import inv


_BASE_TF = 'base_link'
_MAP_TF = 'map'
_HEAD_TF = 'head_rgbd_sensor_link'
_ARM_LIFT_TF = 'arm_lift_link'
_ORIGIN_TF = 'head_rgbd_sensor_link' 
ARM_LENGTH = 0.47625 #0.35; # distance from shoulder joint to wrist joints. measured on the robot
HAND_LENGTH = 0.1524 # distance from end of arm to gripper pads
MAX_ARM_LIFT=0.69 #joint limit of arm lift joint 
OBJECT_LIFT_OFFSET = 0.04
MAX_V = 0.2
MIN_V = 0.05
ERROR_THRESHOLD = 0.01
SLOW_V_CUTOFF = ERROR_THRESHOLD + .01
V_SLOPE_ERROR = (MAX_V-MIN_V)/(1-SLOW_V_CUTOFF)
V_B_ERROR = MIN_V-V_SLOPE_ERROR * SLOW_V_CUTOFF

class GraspAction(object):

	def __init__(self, robot):

		self.robot = robot 
		self.gripper_state=True
		self.print_count=0

		self.target_pose=PoseStamped()
		self.robot_pos=PoseStamped()


		# in the base link frame
		self.target_backup=PoseStamped()
		self.target_backup.pose.position.x=-0.2
		self.target_backup.pose.position.y=0.0
		self.target_backup.pose.position.z=0.0
		self.target_backup.pose.orientation.x=0.0
		self.target_backup.pose.orientation.y=0.0
		self.target_backup.pose.orientation.z=0.0
		self.target_backup.pose.orientation.w=0.0
		self.target_backup.header.frame_id=_BASE_TF


		self.listener=tf.TransformListener()

		self.vel_pub = rospy.Publisher('/hsrb/command_velocity', geometry_msgs.msg.Twist, queue_size=10)

		global_pose_topic = 'global_pose'
		rospy.Subscriber(global_pose_topic, PoseStamped, self.pose_callback)
		joint_states_topic = 'hsrb/joint_states'
		rospy.Subscriber(joint_states_topic, JointState, self.joint_state_Cb)

		while not rospy.is_shutdown():
			try:
				self.body=self.robot.try_get('whole_body')
				self.gripper = self.robot.try_get('gripper')
				# self.base=self.robot.try_get('omni_base')
				self.open_gripper()
				self.body.move_to_neutral()
				break 
			except(exceptions.ResourceNotFoundError, exceptions.RobotConnectionError) as e:
				rospy.logerr("Failed to obtain resource: {}\nRetrying...".format(e))

		# navigation client to move the base 
		self.navi_cli = actionlib.SimpleActionClient('/move_base/move', MoveBaseAction)

		self._as = actionlib.SimpleActionServer('pickUpaction', hsr_manipulation_2019.msg.pickUpAction, execute_cb=self.pickUp, auto_start=False)
		
		self._putdown_as = actionlib.SimpleActionServer('putDownaction', hsr_manipulation_2019.msg.putDownAction, execute_cb=self.putDown, auto_start=False)

		self._as.start()
		self._putdown_as.start()

	def compute_error(self, target_map):

		# self.listener.waitForTransform(_ORIGIN_TF,_BASE_TF,rospy.Time(),rospy.Duration(1.0))
		# self.target_pose_base = self.listener.transformPose(_BASE_TF, target_map)

		self.listener.waitForTransform(_MAP_TF,_BASE_TF,rospy.Time(),rospy.Duration(1.0))
		self.target_pose_base = self.listener.transformPose(_BASE_TF, target_map)
		self.curr_pose_base = self.listener.transformPose(_BASE_TF, self.robot_pos)

		# target location
		target_x = self.target_pose_base.pose.position.x 
		target_y = self.target_pose_base.pose.position.y 
		target_z = self.target_pose_base.pose.position.z
		target_rx = self.target_pose_base.pose.orientation.x
		target_ry = self.target_pose_base.pose.orientation.y
		target_rz = self.target_pose_base.pose.orientation.z
		target_rw = self.target_pose_base.pose.orientation.w

		# current location
		curr_x = self.curr_pose_base.pose.position.x 
		curr_y = self.curr_pose_base.pose.position.y
		curr_z = self.curr_pose_base.pose.position.z
		curr_rx = self.curr_pose_base.pose.orientation.x  
		curr_ry = self.curr_pose_base.pose.orientation.y
		curr_rz = self.curr_pose_base.pose.orientation.z
		curr_rw = self.curr_pose_base.pose.orientation.w   

		error_x = target_x-curr_x
		error_y = target_y-curr_y
		error_x_norm = LA.norm(error_x) #+ LA.norm(target_y-curr_y) + LA.norm(target_z-curr_z) + LA.norm(target_rx - curr_rx) + LA.norm(target_ry-curr_ry) + LA.norm(target_rz - curr_rz) + LA.norm(target_rw - curr_rw)		
		error_y_norm = LA.norm(error_y)

		if self.print_count == 500: 
			# print "target_x", target_x
			# print "curr_x",curr_x
			# print "target_y", target_y
			# print "curr_y",curr_y
			# print ""
			print "current_pose_map \n"
			print self.robot_pos

			self.print_count = 0 
		else: 
			self.print_count+=1 
		
		return error_x_norm, error_y_norm, error_x, error_y  


	def get_vel_command(self, error_norm, error): 

		if error_norm > SLOW_V_CUTOFF:
			v = min(MAX_V, V_SLOPE_ERROR*error_norm+V_B_ERROR)
		else:
			v = MIN_V


		vel_command = np.sign(error) * v

		return vel_command 		


	def track_motion(self, target_map):
 
		error_x_norm, error_y_norm, error_x, error_y  = self.compute_error(target_map)
		original_error_x = error_x_norm 
		original_error_y = error_y_norm 
		prev_error_x = error_x_norm 
		prev_error_y = error_y_norm 

		print "original error x, y", original_error_x, original_error_y
		

		while(error_x_norm > ERROR_THRESHOLD or error_y_norm > ERROR_THRESHOLD): 

			# TODO consider twist errors?  
			tw = geometry_msgs.msg.Twist()
			tw.linear.x = 0
			tw.linear.y = 0

			if error_x_norm > ERROR_THRESHOLD:
				tw.linear.x = self.get_vel_command(error_x_norm, error_x)
			if error_y_norm > ERROR_THRESHOLD:
				tw.linear.y = self.get_vel_command(error_y_norm, error_y)

			prev_error_x = error_x_norm 
			prev_error_y = error_y_norm 			

			# the velocity 
			self.vel_pub.publish(tw)

			# if self.print_count == 1000: 
			# 	print "tw.linear.x", tw.linear.x
			# 	print "tw.linear.y", tw.linear.y 
			# 	print "error_x", error_x_norm
			# 	print "error_y", error_y_norm
			# 	print "prev_error_x", prev_error_x
			# 	print "prev_error_y", prev_error_y


			error_x_norm, error_y_norm, error_x, error_y = self.compute_error(target_map)

		return 	

	def get_target_pose(self, goal_rgbd_sensor_link):
			
		self.target_pose=goal_rgbd_sensor_link.target_pose

		# account for length of arm fully extended
		self.target_pose.pose.position.z=self.target_pose.pose.position.z-ARM_LENGTH-HAND_LENGTH

		self.listener.waitForTransform(_ORIGIN_TF,_MAP_TF,rospy.Time(),rospy.Duration(2.0))
		target_pose_map = self.listener.transformPose(_MAP_TF,self.target_pose)
		self.listener.waitForTransform(_ORIGIN_TF,_ARM_LIFT_TF,rospy.Time(),rospy.Duration(2.0))
		target_pose_arm_lift = self.listener.transformPose(_ARM_LIFT_TF,self.target_pose)

		print "target_pose_map \n"
		print target_pose_map

		return (target_pose_map, target_pose_arm_lift)

	def backUp(self):

		self.listener.waitForTransform(_BASE_TF,_MAP_TF,rospy.Time(),rospy.Duration(2.0))
		# target position to back up to in the map frame 
		self.target_backup_map = self.listener.transformPose(_MAP_TF,self.target_backup)

		self.track_motion(self.target_backup_map)

		rospy.loginfo("back up complete")

		return 


	# TODO: figure out assumptions. Will the robot be directly in front of the object?
	def pickUp(self, goal):

		target_pose_map, target_pose_arm_lift = self.get_target_pose(goal)

		# make sure gripper is open
		self.open_gripper()
		self.body.move_to_neutral()
		self.gripper_state=True
		rospy.loginfo("open_gripper")

		obj_arm_lift_link = target_pose_arm_lift.pose.position.z
		obj_arm_flex_joint = -1.57
		
		self.body.move_to_joint_positions({'arm_flex_joint': obj_arm_flex_joint, 'wrist_flex_joint': 0.0, 'arm_lift_joint':obj_arm_lift_link})

		print "obj_arm_lift_link",obj_arm_lift_link

		self.track_motion(target_pose_map)

		print "forward motion complete"
		
		rospy.sleep(2.0)

		print "current_pose_map \n"
		print self.robot_pos

		self.close_gripper()
		self.gripper_state=False
		rospy.loginfo("close_gripper")

		arm_obj_lift_val = min(self.cur_arm_lift+OBJECT_LIFT_OFFSET , MAX_ARM_LIFT)

		self.body.move_to_joint_positions({'arm_lift_joint':arm_obj_lift_val})

		self.backUp()

		self.body.move_to_neutral()

		self._as.set_succeeded()

	def putDown(self, goal):

		target_pose_map, target_pose_arm_lift = self.get_target_pose(goal)

		print "target_pose_map"
		print target_pose_map
		
		goal_obj_arm_lift_link = target_pose_arm_lift.pose.position.z	

		arm_obj_lift_val = min(goal_obj_arm_lift_link+OBJECT_LIFT_OFFSET , MAX_ARM_LIFT)
		obj_arm_flex_joint = -1.57
		
		self.body.move_to_joint_positions({'arm_flex_joint': obj_arm_flex_joint, 'wrist_flex_joint': 0.0, 'arm_lift_joint':arm_obj_lift_val})

		self.track_motion(target_pose_map)
		
		rospy.sleep(2.0)	
		
		self.body.move_to_joint_positions({'arm_lift_joint':goal_obj_arm_lift_link })
		
		self.open_gripper()

		arm_obj_lift_val = min(self.cur_arm_lift+OBJECT_LIFT_OFFSET , MAX_ARM_LIFT)

		self.body.move_to_joint_positions({'arm_lift_joint':arm_obj_lift_val})

		self.backUp()
		
		self.body.move_to_neutral()
		
		self._putdown_as.set_succeeded()

	def joint_state_Cb(self, msg):
		self.cur_arm_flex=msg.position[0]
		self.cur_arm_lift=msg.position[1]
		self.cur_arm_roll=msg.position[2]
		self.cur_wrist_roll=msg.position[12]
		self.cur_wrist_flex=msg.position[11]

	def pose_callback(self,msg):
		self.robot_pos.pose=msg.pose 
		self.robot_pos.header.frame_id=_MAP_TF

	def open_gripper(self,to_width=1.2):
		self.gripper.command(to_width)

	def close_gripper(self, to_width=-0.01):
		# self.gripper.grasp(to_width)
		self.gripper.apply_force(1.0)

	# def navigation_action(goal_x,goal_y,goal_yaw):
	# 	pose = PoseStamped()
	# 	pose.header.stamp = rospy.Time.now()
	# 	pose.header.frame_id = "map"
	# 	pose.pose.position = Point(goal_x, goal_y, 0)
	# 	quat = tf.transformations.quaternion_from_euler(0, 0, goal_yaw)
	# 	pose.pose.orientation = Quaternion(*quat)

	# 	goal = MoveBaseGoal()
	# 	goal.target_pose = pose

	# 	# send message to the action server
	# 	self.navi_cli.send_goal(goal)

	# 	# wait for the action server to complete the order
	# 	self.navi_cli.wait_for_result()

	# 	# print result of navigation
	# 	result_action_state = self.navi_cli.get_state()

	# 	return #result_action_state 

if __name__ == '__main__':
	robot = Robot()
	rospy.loginfo("Initializing givepose server")
	server=GraspAction(robot)
	rospy.loginfo("grasp_action_server created")
	rospy.spin()

