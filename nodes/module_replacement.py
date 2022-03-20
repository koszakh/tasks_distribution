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
from gazebo_communicator.Repairbot import Repairbot
from gazebo_communicator.Brokenbot import Brokenbot
import task_management.EquipmentDelivery as ed
from task_management.ModuleReplacement import ModuleReplacement 
import random
from path_planning.MovementManager import MovementManager

rospy.init_node('module_replacement')
print(sys.version)
names = ['sim_p3at' + str(i) for i in range(1, t_const.ROBOTS_COUNT + 1)]
mh = prepare_map_handler()
robots = {}
modules_count = t_const.NUMBER_OF_MODULES
avg_x = np.mean([mh.min_x, mh.max_x])
avg_y = np.mean([mh.min_y, mh.max_y])

s_x = const.S_X_OFFSET
s_y = const.S_Y_OFFSET
g_x = const.G_X_OFFSET
g_y = const.G_Y_OFFSET

start = (avg_x + s_x, avg_y + s_y)
goal = (avg_x + g_x, avg_y + g_y)
broken_r_name = random.choice(names)

for name in (names):
	
	robot_pos, orient = mh.get_start_pos(start[0], start[1], const.START_DIST_OFFSET)
	gc.spawn_worker(name, robot_pos, orient)
	if name == broken_r_name:
		broken_robot = Brokenbot(name)
		broken_robot.movable = False # Can move or not
	else:
		robots[name] = Repairbot(name)


module_p_ids = mh.get_random_points_in_area(goal[0], goal[1], const.GOAL_DIST_OFFSET, modules_count)
mr = ModuleReplacement(robots, broken_robot, module_p_ids, mh)
ta_dict = mr.assign_robots_to_modules()
full_robots = robots
full_robots[broken_r_name] = broken_robot
for r_key in list(ta_dict.keys()):

	t_key = ta_dict[r_key]
	path_to_equip, path_to_meet_p, br_path = mr.plan_replacement_path(r_key, t_key)
	repair_robot = robots[r_key]
	repair_robot.set_repair_data(path_to_equip, path_to_meet_p)
	broken_robot.set_meet_p_data(br_path)
	mm = MovementManager(mh, full_robots)
	mm.start()
	while not broken_robot.finished and not repair_robot.finished:

		pass

	del mm
rospy.spin()

