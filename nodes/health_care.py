#!/usr/bin/env python3

import rospy
from time import sleep
import sys
import gazebo_communicator.GazeboCommunicator as gc
from path_planning.Heightmap import prepare_map_handler
import task_management.Clusterization as cl
import path_planning.PathPlanner as pp
import path_planning.Constants as const
from path_planning.Point import Vector2d, Point
import numpy as np
from gazebo_communicator.Deliverybot import Deliverybot
from gazebo_communicator.Nodebot import Nodebot
import task_management.EquipmentDelivery as ed
import task_management.TaskConstants as t_const
import task_management.NetworkCreation as nc
import random
from path_planning.MovementManager import MovementManager
import copy

def create_network(r_keys, mh, beacon_pos, group_pos, group_route, mm):

	robots = {}	
	for key in r_keys:
		robots[name] = Nodebot(name)
	nc_obj = nc.NetworkCreation(robots, mh, beacon_pos, group_pos, group_route)
	net_dict = nc_obj.create_network()
		
	if net_dict:
		#active_robots = {key: robots[key] for key in list(robots.keys()) if net_dict.get(key)}
		mm.add_robots(robots)
		mm.prepare_network_mission(net_dict)
		mm.start()
	else:
		print('Network cannot be realized!')

def medicine_delivery(names, quadrotors, group_pos, group_route, mh, goal):
	
	robots = {}
	for name in (names):
		robots[name] = Deliverybot(name)
		
	med_count = t_const.EQUIP_COUNT	
	target_ids = mh.get_random_ids_in_area(goal[0], goal[1], const.GOAL_DIST_OFFSET, t_const.EQUIP_COUNT)
	if group_route:
		
		ed_planner = ed.EquipmentDelivery(robots, quadrotors, target_ids, group_pos, group_route, mh)
		paths_to_equip, paths_to_group = ed_planner.plan_equipment_delivery()
		all_robots = ed_planner.all_robots
		inactive_r_keys = [r_key for r_key in list(all_robots.keys()) if isinstance(all_robots[r_key], Deliverybot) and not paths_to_equip.get(r_key)]
		for key in inactive_r_keys:
		
			robot = all_robots[key]
			robot.unregister_subs()
			del robot
		mm = MovementManager(mh, robots)
		mm.prepare_delivery_mission(paths_to_equip, paths_to_group)
		return inactive_r_keys, mm
		
	else:

		print('Tourists route are unavailable for shipping.')
		return {}
		
def busy_robots_simulation(names, mh):

	busy_robots = {}
	tmp_names = copy.copy(names)
	tmp_target_ids = mh.get_random_ids_in_area(goal[0], goal[1], const.GOAL_DIST_OFFSET, t_const.BUSY_ROBOTS_COUNT)
	
	for i in range(t_const.BUSY_ROBOTS_COUNT):

		r_key = random.choice(tmp_names)
		tmp_names.remove(r_key)
		#print(len(tmp_target_ids), tmp_target_ids)
		t_key = random.choice(list(tmp_target_ids.keys()))
		rand_gr_key = mh.get_random_free_id()
		tmp_target_ids.pop(t_key)
		robot = Deliverybot(r_key)
		r_pos = robot.get_robot_position()
		r_id = mh.get_nearest_vertice_id(r_pos.x, r_pos.y)
		r_vect = robot.get_robot_orientation_vector()
		eq_path, path_cost = mh.find_path(r_id, t_key, r_vect)
		if eq_path:
			last_vect = pp.get_end_vect(eq_path)
			gr_path, path_cost = mh.find_path(t_key, rand_gr_key, last_vect)
			if gr_path:
				robot.set_delivery_data(eq_path, gr_path)
				busy_robots[r_key] = robot
	
	sleep(1)
	for key in list(busy_robots.keys()):
	
		robot = busy_robots[key]
		robot.start()

	return busy_robots
		

rospy.init_node('health_care')
sos_signal = ["rescuers_call", "medicines_delivery", "doctor_video_call"]
mh = prepare_map_handler()
avg_x = np.mean([mh.min_x, mh.max_x])
avg_y = np.mean([mh.min_y, mh.max_y])

s_x = const.S_X_OFFSET
s_y = const.S_Y_OFFSET
g_x = const.G_X_OFFSET
g_y = const.G_Y_OFFSET

start = (avg_x + s_x, avg_y + s_y)
goal = (avg_x + g_x, avg_y + g_y)

names = ['sim_p3at' + str(i) for i in range(1, t_const.ROBOTS_COUNT + 1)]
for name in names:
		
		robot_pos, orient = mh.get_start_pos(start[0], start[1], const.START_DIST_OFFSET)
		gc.spawn_worker(name, robot_pos, orient)

busy_robots = busy_robots_simulation(names, mh)
quadrotors = {}
group_pos_id = mh.get_random_free_id()
group_pos = mh.heightmap[group_pos_id]
#group_goal_id = mh.get_random_free_id()
for key in list(busy_robots.keys()):

	robot = busy_robots[key]
	robot.stop()
	robot.unregister_subs()
	del robot
del busy_robots
group_route = [group_pos]#mh.find_tourists_path(group_pos_id, group_goal_id)
if sos_signal.__contains__("rescuers_call"):

	pass

if sos_signal.__contains__("medicines_delivery"):

	inactive_keys, mm = medicine_delivery(names, quadrotors, group_pos, group_route, mh, goal)

if sos_signal.__contains__("doctor_video_call"):

	beacon_id = mh.get_random_free_id()
	beacon_pos = mh.heightmap[beacon_id]	
	create_network(inactive_keys, mh, beacon_pos, group_pos, group_route, mm)

rospy.spin()

