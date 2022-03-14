# Module replacement module

import numpy as np
import task_management.Clusterization as cl
import task_management.DDPSO as pso_opt
import task_management.TaskConstants as const
from gazebo_communicator.Deliverybot import Deliverybot
#import gazebo_communicator.GazeboCommunicator as gc
from path_planning.PathPlanner import get_path_length, get_end_vect
from collections import Counter

class ModuleReplacement:

	def __init__(self, robots, broken_robot, mp_poses, mh):

		self.robots = robots
		self.broken_robot = broken_robot
		self.mp_poses = mp_poses
		self.mh = mh
		self.paths, self.path_costs = self.mh.calc_paths(self.robots, self.mp_poses)

	def get_current_robots_mas(self, names_mas):
	
		r_mas = []
		for name in names_mas:
		
			robot = self.robots[name]
			r_mas.append(robot)
			
		return r_mas
		
	def assign_robots_to_modules(self):

		cc = cl.CustomClustering(self.robots, self.mp_poses, self.mh, self.path_costs)
		clust_dict = cc.start_clustering()
		#print('clust_dict: ' + str(clust_dict))
		ta_dict = {}
		for t_key in clust_dict:

			if len(clust_dict[t_key]) > 0:

				current_robots = self.get_current_robots_mas(clust_dict[t_key])
				pso_obj = pso_opt.DDPSO(current_robots, self.mp_poses[t_key], self.path_costs)
				solution = pso_obj.run_pso()
				r_name = current_robots[solution[0]].name
				ta_dict[r_name] = t_key

			else:
			
				print('The module is in an unreachable zone!')
				return {}
				
		return ta_dict
		

	def func(self):
		
		for r_key in ta_dict.keys():

			t_key = ta_dict[r_key]
			path_to_equip, path_to_t_group, total_time = self.plan_replacement_path(r_key, t_key)
			if total_time > mission_time:
			
				mission_time = total_time

		return mission_time, ta_dict
			
	def plan_replacement_path(self, r_key, t_key):

		robot = self.robots[r_key]
		path_key = pso_opt.get_path_key(r_key, t_key)
		path_to_equip = self.paths[path_key]
		r_pos_key = path_to_equip[0].id
		eq_path_len = get_path_length(path_to_equip)
		time_to_equip = eq_path_len / robot.ms
		last_vect = get_end_vect(path_to_equip)
		if self.broken_robot.movable:
		
			meet_p_key, br_path = self.get_module_replacement_pos(t_key, time_to_equip)
		
		else:
		
			br_pos = self.broken_robot.get_robot_position()
			meet_p_key = self.mh.get_nearest_vertice_id(br_pos.x, br_pos.y)
			br_path = []
			
		path_to_meet_p, path_cost = self.mh.find_path(t_key, meet_p_key, last_vect)
			
		if path_to_meet_p:

			return path_to_equip, path_to_meet_p, br_path

		else:

			return [], [], []
	
	def get_module_replacement_pos(self, mp_key, time_to_mp):
	
		br_pos = self.broken_robot.get_robot_position()
		br_key = self.mh.get_nearest_vertice_id(br_pos.x, br_pos.y)
		br_vect = self.broken_robot.get_robot_orientation_vector()
		br_path, path_cost = self.mh.find_path(br_key, mp_key, br_vect)
		br_path_len = get_path_length(br_path)
		br_move_time = br_path_len / self.broken_robot.ms
		if br_move_time < time_to_mp:
		
			br_path.pop(-1)
			meet_p_key = br_path[-1].id
			
		else:
		
			while br_move_time > time_to_mp:
			
				br_path.pop(-1)
				br_path_len = get_path_length(br_path)
				br_move_time = br_path_len / self.broken_robot.ms
				
			meet_p_key = br_path[-1].id
		
		return meet_p_key, br_path
	
	def plan_equipment_delivery(self):
	
		active_robots, assignment = self.get_active_robots()
		paths_to_eq = {}
		paths_to_gr = {}
		for r_key in list(assignment.keys()):
		
			t_key = assignment[r_key]
			paths_to_eq[r_key], paths_to_gr[r_key], total_time = self.plan_robot_path(r_key, t_key)

		return paths_to_eq, paths_to_gr
	
	def calc_est_time_to_group(self, path_to_eq, time_to_eq, ms):
	
		eq_pos = path_to_eq[-1]
		for p in self.group_route:

			dist = p.get_distance_to(eq_pos)
			travel_time = (dist / ms) * const.GR_ROBOT_TIME_COEF
			full_time = time_to_eq + travel_time
			gr_pos = self.calc_potential_group_pos(full_time)
			p_dist = gr_pos.get_distance_to(p)
			if p_dist < 2:
			
				est_time = full_time
				break
		
		return est_time
		
	def calc_potential_group_pos(self, est_time):
	
		route_copy = self.group_route
		path_len = get_path_length(route_copy)
		est_distance = est_time * const.TOURISTS_GROUP_SPEED
		if est_distance >= path_len:
		
			potential_pos = route_copy[-1]
			
		else:
		
			while est_distance < path_len:
			
				last_p = route_copy[-1]
				pre_last_p = route_copy[-2]
				dist = last_p.get_distance_to(pre_last_p)
				route_copy.pop(-1)
				path_len -= dist
				
			potential_pos = pre_last_p
			
		return potential_pos
