#!/usr/bin/env python3

import rospy
from time import sleep
import sys
import gazebo_communicator.GazeboCommunicator as gc
import gazebo_communicator.GazeboConstants as gc_const
from path_planning.Heightmap import prepare_map_handler
import task_management.Clusterization as cl
import task_management.TaskConstants as t_const
import path_planning.PathPlanner as pp
import path_planning.Constants as const
from path_planning.Point import Vector2d, Point
import numpy as np
from gazebo_communicator.Deliverybot import Deliverybot
import task_management.EquipmentDelivery as ed
import random
from path_planning.MovementManager import MovementManager

rospy.init_node('equipment_delivery')
print(sys.version)
names = ['sim_p3at' + str(i) for i in range(1, t_const.ROBOTS_COUNT + 1)]
mh = prepare_map_handler()
robots = {}
quadrotors = {}
t_count = t_const.EQUIP_COUNT
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
	robots[name] = Deliverybot(name)
	
target_ids = mh.get_random_ids_in_area(goal[0], goal[1], const.GOAL_DIST_OFFSET, t_count)
group_pos_id = mh.get_random_free_id()
ids = mh.get_close_points_list(group_pos_id, const.CLOSE_RADIUS)
points = [mh.heightmap[v_id] for v_id in ids]
gc.visualise_path(points, gc_const.RED_VERTICE_PATH, 'cp')
group_pos = mh.heightmap[group_pos_id]
gc.spawn_sdf_model(group_pos, gc_const.GREEN_VERTICE_PATH, 'gr_p')
group_goal_id = mh.get_random_free_id()
group_route = mh.find_tourists_path(group_pos_id, group_goal_id)
#gc.visualise_path(group_route, gc_const.BLUE_VERTICE_PATH, 'gr_p')

if group_route:

	ed_planner = ed.EquipmentDelivery(robots, quadrotors, target_ids, group_pos, group_route, mh)
	paths_to_equip, paths_to_group = ed_planner.plan_equipment_delivery()
	all_robots = ed_planner.all_robots
	active_robots = {r_key: all_robots[r_key] for r_key in list(all_robots.keys()) if isinstance(all_robots[r_key], Deliverybot) and paths_to_equip.get(r_key)}
	mm = MovementManager(mh, robots)
	mm.prepare_delivery_mission(paths_to_equip, paths_to_group)
	mm.start()
	
else:

	print('Tourists route are unavailable for shipping.')
rospy.spin()

