from gazebo_communicator.Robot import Robot
import gazebo_communicator.GazeboCommunicator as gc
import threading as thr
import gazebo_communicator.GazeboConstants as const
import path_planning.Constants as pp_const
from math import fabs
import rospy
import random

class Cleaner(Robot):

	def __init__(self, name):

		thr.Thread.__init__(self)
		self.name = name
		self.init_topics()
		self.pid_delay = rospy.Duration(0, const.PID_NSEC_DELAY)
		self.bt = self.trackers[name]
		self.finished = False
		self.waiting = False
		self.mode = "stop"
		self.dodging = False
		self.manipulator = random.choice([True, False])

	def change_mode(self, mode):
	
		self.mode = mode
		rospy.loginfo("Cleaner " + self.name + " changed mode to: " + str(self.mode))

	def perform_cleaning_mission(self):

		for g_path in self.g_paths:
		
			self.follow_the_route(g_paths)

		
# Moving the robot to a point with a PID controller
# Input
# goal: target point
	def move_with_PID(self, goal):
	
		error = self.get_angle_difference(goal)
		error_sum = 0
		robot_pos = self.get_robot_position()
		old_pos = robot_pos
		
		while robot_pos.get_distance_to(goal) > const.DISTANCE_ERROR and fabs(error) < 90:
		
			old_error = error
			robot_pos = self.get_robot_position()
			error = self.get_angle_difference(goal)
			u, error_sum = self.calc_control_action(error, old_error, error_sum)
			self.movement(self.ms, u)
			self.is_waiting()
			self.is_dodging()
			old_pos = robot_pos
			rospy.sleep(self.pid_delay)

	def set_cleaning_data(self, g_paths):

		if g_paths:
		
			self.g_paths = g_paths
			
		else:
		
			self.g_paths = None



	def run(self):
	
		if self.g_paths:
				
			self.perform_cleaning_mission()
				
		print('Cleaner ' + str(self.name) + ' has finished!')

		self.change_mode("finished")
