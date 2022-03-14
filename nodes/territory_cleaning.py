#!/usr/bin/env python3

import rospy
from time import sleep
import sys
import gazebo_communicator.GazeboCommunicator as gc
from path_planning.Heightmap import prepare_map_handler
import task_management.Clusterization as cl
import task_management.TaskConstants as t_const
import path_planning.PathPlanner as pp
import path_planning.Constants as const
from path_planning.Point import Vector2d, Point
import numpy as np
from gazebo_communicator.Deliverybot import Deliverybot
from gazebo_communicator.Cleaner import Cleaner
import task_management.EquipmentDelivery as ed
from task_management.TerritoryCleaning import TerritoryCleaning
import random
from path_planning.MovementManager import MovementManager

rospy.init_node('territory_cleaning')
names = ['sim_p3at' + str(i) for i in range(1, t_const.ROBOTS_COUNT + 1)]
mh = prepare_map_handler()
robots = {}
garbage_count = t_const.AMOUNT_OF_GARBAGE
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
garbage_ids = mh.get_random_ids_in_area(goal[0], goal[1], const.GARBAGE_DIST_OFFSET, garbage_count)
group_pos_id = mh.get_random_free_id()
group_pos = mh.heightmap[group_pos_id]
group_goal_id = mh.get_random_free_id()
group_route = mh.find_tourists_path(group_pos_id, group_goal_id)
if group_route:
	print('\nEquipment delivery started!')
	ed_planner = ed.EquipmentDelivery(robots, {}, target_ids, group_pos, group_route, mh)
	paths_to_equip, paths_to_group = ed_planner.plan_equipment_delivery()
	print(paths_to_equip.keys())
	all_robots = ed_planner.all_robots
	mm = MovementManager(mh, robots)
	mm.set_garbage_poses(garbage_ids)
	mm.prepare_delivery_mission(paths_to_equip, paths_to_group)
	mm.vis_garbage()
	mm.start()

while not mm.mm_finished:
	pass
garbage_poses, detected_garbage = mm.get_garbage_ids()
print('\nTerritory cleaning started!')
tc = TerritoryCleaning(robots, detected_garbage, mh)
g_paths = tc.distribute_garbage()
for r_key in g_paths:
	del robots[r_key]
	robot = Cleaner(r_key)
	robots[r_key] = robot
del mm
mm = MovementManager(mh, robots)
mm.set_garbage_poses(garbage_poses)
mm.prepare_cleaning_mission(g_paths)
mm.start()
rospy.spin()

