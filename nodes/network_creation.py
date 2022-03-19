#!/usr/bin/env python3

import rospy
from time import sleep
import sys
import gazebo_communicator.GazeboCommunicator as gc
import gazebo_communicator.GazeboConstants as gc_const
from gazebo_communicator.Nodebot import Nodebot
from path_planning.Heightmap import prepare_map_handler
from path_planning.PathPlanner import get_path_length
import path_planning.Constants as const
from path_planning.Point import Vector2d, Point
import numpy as np
import task_management.NetworkCreation as nc
import task_management.TaskConstants as t_const
import random
from path_planning.MovementManager import MovementManager



def create_network(robots, mh, beacon_pos, group_pos, group_route):

	nc_obj = nc.NetworkCreation(robots, mh, beacon_pos, group_pos, group_route)
	net_dict = nc_obj.create_network()
	if net_dict:
		#active_robots = {key: robots[key] for key in list(robots.keys()) if net_dict.get(key)}
		mm = MovementManager(mh, robots)
		mm.prepare_network_mission(net_dict)
		mm.start()
	else:
		print('Network cannot be realized!')

rospy.init_node('network_creation')
mh = prepare_map_handler()
group_pos_id = mh.get_random_free_id()
group_pos = mh.heightmap[group_pos_id]
group_goal_id = mh.get_random_free_id()
group_route = mh.find_tourists_path(group_pos_id, group_goal_id)
#gc.visualise_path(group_route, gc_const.BLUE_VERTICE_PATH, 'gr_p')
gr_path_len = get_path_length(group_route)
print('\n Tourists group path length: ' + str(gr_path_len))
beacon_id = mh.get_random_free_id()
beacon_pos = mh.heightmap[beacon_id]
#gc.spawn_sdf_model(beacon_pos, gc_const.GREEN_VERTICE_PATH, 'beacon_pos')
names = ['sim_p3at' + str(i) for i in range(1, t_const.ROBOTS_COUNT + 1)]
robots = {}
avg_x = np.mean([mh.min_x, mh.max_x])
avg_y = np.mean([mh.min_y, mh.max_y])

s_x = const.S_X_OFFSET
s_y = const.S_Y_OFFSET
g_x = const.G_X_OFFSET
g_y = const.G_Y_OFFSET

start = (avg_x + s_x, avg_y + s_y)
goal = (avg_x + g_x, avg_y + g_y)

for name in (names):
		
		robot_pos, orient = mh.get_start_pos(start[0], start[1], const.START_DIST_OFFSET)
		gc.spawn_worker(name, robot_pos, orient)
		robots[name] = Nodebot(name)
		
create_network(robots, mh, beacon_pos, group_pos, group_route)

rospy.spin()

