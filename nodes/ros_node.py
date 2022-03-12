#!/usr/bin/env python

import rospy
from time import sleep
import sys
import gazebo_communicator.GazeboCommunicator as gc
from path_planning.Heightmap import Heightmap
#import task_management.TerritoryCleaning as tc
import task_management.Clusterization as cl
import path_planning.PathPlanner as pp
import path_planning.Constants as const
import numpy as np
from gazebo_communicator.Robot import Robot

def get_min_dist(p, points):

	min_dist = float('inf')
	
	for n_p in points:
	
		dist = p.get_distance_to(n_p)
		
		if dist < min_dist:
		
			min_dist = dist
			
	return min_dist

def delete_intermediate_points(path, cut_step):

	path_copy = copy.copy(path)

	for i in range(len(path) - 1):

		if (i % cut_step > 0):

			if path[i] in path_copy:

				path_copy.remove(path[i])

	return path_copy
	 
def convert_to_path(msg):
	path = []
	for point_msg in msg.path:
		p = convert_to_point(point_msg)
		path.append(p)
	return path

def convert_to_point(msg):
	x = msg.x
	y = msg.y
	z = msg.z
	point = Point(x, y, z)
	return point

def make_pose_msg(state, orient):
	msg = Pose()
	msg.position.x = state.x
	msg.position.y = state.y
	msg.position.z = state.z
	if orient:
		msg.orientation.x = orient[0]
		msg.orientation.y = orient[1]
		msg.orientation.z = orient[2]
	return msg

rospy.init_node('ros_node')
sleep(1)
names = ['sim_p3at' + str(i) for i in range(1, 6)]
hm = Heightmap(const.HEIGHTMAP_SDF_PATH)
hmap, l_scale, w_scale, x_step, y_step, step_count = hm.prepare_heightmap()
mh = pp.PathPlanner(hmap, l_scale, w_scale, x_step, y_step, step_count)
mh.gridmap_preparing()
robots = {}
t_count = 3
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
	robots[name] = Robot(name)
	
target_ids = mh.get_random_ids_in_area(goal[0], goal[1], const.GOAL_DIST_OFFSET, t_count)
cc = cl.CustomClustering(robots, target_ids, mh)
cc.run_clusterization()
print('Finish!')
rospy.spin()

