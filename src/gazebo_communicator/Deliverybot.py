from gazebo_communicator.Robot import Robot
import gazebo_communicator.GazeboCommunicator as gc
import threading as thr
import gazebo_communicator.GazeboConstants as const
import path_planning.Constants as pp_const
from math import fabs
import rospy
import random

class Deliverybot(Robot):

	def __init__(self, name):

		thr.Thread.__init__(self)
		self.name = name
		self.init_topics()
		self.pid_delay = rospy.Duration(0, const.PID_NSEC_DELAY)
		self.paths = []
		self.workpoints = None
		self.charge_points = None
		self.finished = False
		self.waiting = False
		self.mode = "stop"
		self.dodging = False
		self.ms = const.MOVEMENT_SPEED
		self.to_equip_path = []
		self.movable = True
		self.manipulator = random.choice([True, False])

	def change_mode(self, mode):
	
		self.mode = mode
		rospy.loginfo("Deliverybot " + self.name + " changed mode to: " + str(self.mode))

	def perform_delivery_mission(self):

		self.follow_the_route(self.to_equip_path)
		self.follow_the_route(self.to_group_path)

		
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


	def get_robot_battery_level(self, name):
	
		bt = self.trackers[name]
		b_level = bt.battery
		return b_level

	def get_battery_level(self):

		b_level = self.get_robot_battery_level(self.name)
		return b_level
		
	def print_battery_level(self):
		
		b_level = self.get_battery_level()
		rospy.loginfo("Deliverybot " + self.name + " current battery level: " + str(b_level))

	def set_delivery_data(self, equip_path, group_path):

		if equip_path:
		
			self.to_equip_path = equip_path
			self.to_group_path = group_path
			
		else:
			self.to_equip_path = None



	def run(self):
	
		if self.to_equip_path:
				
			self.perform_delivery_mission()	
			print('Deliverybot ' + str(self.name) + ' has finished!')

		self.change_mode("finished")
