# Equipment delivery module

import numpy as np
import task_management.Clusterization as cl
import task_management.DDPSO as pso_opt
import task_management.TaskConstants as const
from gazebo_communicator.Deliverybot import Deliverybot
import gazebo_communicator.GazeboConstants as gc_const
import gazebo_communicator.GazeboCommunicator as gc
import path_planning.Constants as p_const
from path_planning.PathPlanner import get_path_length, get_end_vect
from collections import Counter

class EquipmentDelivery:

	def __init__(self, robots, quadrotors, eq_poses, group_pos, group_route, mh):

		self.ground_robots = robots
		self.quadrotors = quadrotors
		self.all_robots = {**self.ground_robots, **self.quadrotors}
		self.eq_poses = eq_poses
		self.group_pos = group_pos
		self.group_route = group_route
		self.mh = mh
		self.occupied_cells = {}
		self.paths, self.path_costs = self.mh.calc_paths(self.all_robots, self.eq_poses)

	def get_current_robots_mas(self, names_mas):
	
		r_mas = []
		for name in names_mas:
		
			robot = self.all_robots[name]
			r_mas.append(robot)
			
		return r_mas

	def get_active_robots(self):

		ground_time, gr_assignment = self.calc_mission_time(self.ground_robots)
		print('\n >Estimated ground shipping time: ' + str(ground_time))
		air_time, air_assignment = self.calc_mission_time(self.quadrotors)
		print('\n >Estimated air shipping time: ' + str(air_time))
		combo_time, combo_assignment = self.calc_mission_time(self.all_robots)
		print('\n >Estimated combined shipping time: ' + str(combo_time))
		if ground_time < float('inf'):
		
			print('Ground shipping method selected.')
			active_robots = self.ground_robots
			active_assignment = gr_assignment
			
		elif combo_time < float('inf'):
		
			print('Combined shipping method selected')
			active_robots = self.all_robots
			active_assignment = combo_assignment
			
		else:
		
			print('Selected air shipping method')
			active_robots = self.quadrotors
			active_assignment = air_assignment
			
		return active_robots, active_assignment
		
	def calc_mission_time(self, robots):

		cc = cl.CustomClustering(robots, self.eq_poses, self.mh, self.path_costs)
		clust_dict = cc.start_clustering()
		#print('clust_dict: ' + str(clust_dict))
		ta_dict = {}
		for t_key in clust_dict:

			if len(clust_dict[t_key]) > 0:

				current_robots = self.get_current_robots_mas(clust_dict[t_key])
				pso_obj = pso_opt.DDPSO(current_robots, self.eq_poses[t_key], self.path_costs)
				solution = pso_obj.run_pso()
				r_name = current_robots[solution[0]].name
				ta_dict[r_name] = t_key

			else:
			
				mission_time = float('inf')
		
		mission_time = 0
		self.occupied_cells = {}
		for r_key in ta_dict.keys():

			t_key = ta_dict[r_key]
			path_to_equip, path_to_t_group, total_time = self.plan_robot_path(r_key, t_key)
			if total_time > mission_time:
			
				mission_time = total_time

		return mission_time, ta_dict
			
	def plan_robot_path(self, r_key, t_key):

		robot = self.all_robots[r_key]
		path_key = pso_opt.get_path_key(r_key, t_key)
		path_to_equip = self.paths[path_key]
		eq_path_len = get_path_length(path_to_equip)
		time_to_equip = eq_path_len / robot.ms
		last_vect = get_end_vect(path_to_equip)
		est_time_to_gr = self.calc_est_time_to_group(path_to_equip, time_to_equip, robot.ms)
		est_time = time_to_equip + est_time_to_gr
		gr_pos = self.calc_potential_group_pos(est_time)
		gr_key = gr_pos.id
		self.occupied_cells[gr_key] = gr_pos
		ids = self.mh.get_close_points_list(gr_key, p_const.CLOSE_RADIUS)
		occupied_neighbors = {v_id: self.mh.heightmap[v_id] for v_id in ids}
		self.occupied_cells = {**self.occupied_cells, **occupied_neighbors}
		if isinstance(robot, Deliverybot):

			path_to_t_group, path_cost = self.mh.find_path(t_key, gr_key, last_vect)
			
		else:
		
			start_v = self.mh.heightmap[t_key]
			goal_v = self.mh.heightmap[gr_key]
			path_to_t_group = [start_v, goal_v]
			
		if path_to_t_group:

			gr_path_len = get_path_length(path_to_t_group)
			time_to_group = gr_path_len / robot.ms
			total_time = time_to_equip + time_to_group
			return path_to_equip, path_to_t_group, total_time

		else:

			return [], [], float('inf')
	
	def plan_equipment_delivery(self):
	
		active_robots, assignment = self.get_active_robots()
		paths_to_eq = {}
		paths_to_gr = {}
		self.occupied_cells = {}
		for r_key in list(assignment.keys()):
		
			t_key = assignment[r_key]
			paths_to_eq[r_key], paths_to_gr[r_key], total_time = self.plan_robot_path(r_key, t_key)
			gr_pos = paths_to_gr[r_key][-1]
			gc.spawn_sdf_model(gr_pos, gc_const.GREEN_VERTICE_PATH, 'v-' + str(r_key) + '-' + str(gr_pos.id))
			
		#occ_p = [self.mh.heightmap[v_id] for v_id in self.occupied_cells]
		#gc.visualise_path(occ_p, gc_const.RED_VERTICE_PATH, 'occ_p')

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
		pre_last_p = route_copy[-1]
		if est_distance >= path_len and not self.occupied_cells.get(pre_last_p.id):
		
			potential_pos = pre_last_p
			
		else:
		
			while est_distance < path_len or self.occupied_cells.get(pre_last_p.id):
			
				last_p = route_copy[-1]
				pre_last_p = route_copy[-2]
				dist = last_p.get_distance_to(pre_last_p)
				route_copy.pop(-1)
				path_len -= dist
				
			potential_pos = pre_last_p
			
		return potential_pos
