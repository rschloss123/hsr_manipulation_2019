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


from manip_prelim.msg import *
import math

from numpy import linalg as LA

_ORIGIN_TF = 'base_link' # TODO
_BASE_TF = 'base_link'
_MAP_TF = 'map'

class GraspAction(object):

	def __init__(self, robot):

		self.robot = robot 
		self.gripper_state=True
		self.print_count=0

		self.target_pose=PoseStamped()

		# in the base link frame
		self.target_backup=PoseStamped()
		self.target_backup.pose.position.x=-0.1
		self.target_backup.pose.position.y=0.0
		self.target_backup.pose.position.z=0.0
		self.target_backup.pose.orientation.x=0.0
		self.target_backup.pose.orientation.y=0.0
		self.target_backup.pose.orientation.z=0.0
		self.target_backup.pose.orientation.w=0.0
		self.target_backup.header.frame_id=_ORIGIN_TF


		listener=tf.TransformListener()
		listener.waitForTransform(_BASE_TF,_MAP_TF,rospy.Time(),rospy.Duration(2.0))


		t = listener.getLatestCommonTime(_BASE_TF, _MAP_TF)
		position, quaternion = listener.lookupTransform(_BASE_TF, _MAP_TF, t)
		print "position", position
		print " "
		print "quaternion", quaternion

		self.target_backup_map = listener.transformPose(_MAP_TF,self.target_backup)

		self.error_threshold = 0.01

		self.vel_pub = rospy.Publisher('/hsrb/command_velocity', geometry_msgs.msg.Twist, queue_size=10)

		global_pose_topic = 'global_pose'
		self.global_pose_sub = rospy.Subscriber(global_pose_topic, PoseStamped, self.pose_callback)

		while not rospy.is_shutdown():
			try:
				self.body=self.robot.try_get('whole_body')
				self.gripper = self.robot.try_get('gripper')
				self.base=self.robot.try_get('omni_base')
				self.open_gripper()
				self.body.move_to_neutral()
				break 
			except(exceptions.ResourceNotFoundError, exceptions.RobotConnectionError) as e:
				rospy.logerr("Failed to obtain resource: {}\nRetrying...".format(e))

		# navigation client to move the base 
		self.navi_cli = actionlib.SimpleActionClient('/move_base/move', MoveBaseAction)

		self._as = actionlib.SimpleActionServer('pickUpaction', manip_prelim.msg.pickUpAction, execute_cb=self.pickUp, auto_start=False)
		self._as.start()

	def compute_error(self, target):
		# target location
		target_x = target.pose.position.x 
		target_y = target.pose.position.y 
		target_z = target.pose.position.z
		target_rx = target.pose.orientation.x
		target_ry = target.pose.orientation.y
		target_rz = target.pose.orientation.z
		target_rw = target.pose.orientation.w

		# current location
		curr_x = self.robot_pos.position.x 
		curr_y = self.robot_pos.position.y
		curr_z = self.robot_pos.position.z
		curr_rx = self.robot_pos.orientation.x  
		curr_ry = self.robot_pos.orientation.y
		curr_rz = self.robot_pos.orientation.z
		curr_rw = self.robot_pos.orientation.w   

		error_x = LA.norm(target_x-curr_x) #+ LA.norm(target_y-curr_y) + LA.norm(target_z-curr_z) + LA.norm(target_rx - curr_rx) + LA.norm(target_ry-curr_ry) + LA.norm(target_rz - curr_rz) + LA.norm(target_rw - curr_rw)		
		error_y = LA.norm(target_y-curr_y)

		if self.print_count == 10000: 
			print "error_x", error_x 
			print "error_y", error_y 
			print "target_x", target_x
			print "curr_x",curr_x
			self.print_count = 0 
		else: 
			self.print_count+=1 
		
		return error_x, error_y  


	def track_motion(self, target, backup=False):
		sign_x = 1.0
		sign_y = 1.0

		if backup == True :
			sign_x = -sign_x

		error_x, error_y  = self.compute_error(target)

		while(error_x > self.error_threshold): # or error_y > self.error_threshold:  
			# TODO consider direction of errors
			# TODO consider twist errors?  
			tw = geometry_msgs.msg.Twist()

			# Todo: map link and base link are in opposite directions. Need to access the transform
			# if target.pose.position.y > self.robot_pos.position.y: 
			# 	sign_y = -sign_y  

			if error_x > self.error_threshold:
				tw.linear.x = sign_x * 0.05 

			if self.print_count == 10000:
				print "tw.linear.x", tw.linear.x


			# TODO
			# if error_y > self.error_threshold:
			# 	tw.linear.y = sign_y * 0.1

			self.vel_pub.publish(tw)

			error_x, error_y = self.compute_error(target)

		return 		

	# TODO: figure out assumptions. Will the robot be directly in front of the object?
	def pickUp(self, goal):

		self.target_pose=goal.target_pose

		listener=tf.TransformListener()
		listener.waitForTransform(_ORIGIN_TF,_MAP_TF,rospy.Time(),rospy.Duration(2.0))
		target_pose_map = listener.transformPose(_MAP_TF,self.target_pose)

		# make sure gripper is open
		self.open_gripper()
		self.body.move_to_neutral()
		self.gripper_state=True
		rospy.loginfo("open_gripper")

		# calculate transform for arm
		# TODO : add arm height to action and do transformation
		obj_arm_lift_link = 0.2
		
		self.body.move_to_joint_positions({'arm_flex_joint': -1.57, 'wrist_flex_joint': 0.0, 'arm_lift_joint':obj_arm_lift_link})

		self.track_motion(target_pose_map,backup=False)
		rospy.sleep(2.0)
		self.close_gripper()
		self.gripper_state=False
		rospy.loginfo("close_gripper")


		self.track_motion(self.target_backup_map,backup=True)

		rospy.loginfo("back up complete")
		

		self._as.set_succeeded()

	# TODO
	def setDown():

		self.open_gripper()

	def pose_callback(self,msg):
		self.robot_pos=msg.pose 

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

