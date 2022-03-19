#!/usr/bin/env python3

import rospy
from time import sleep
import sys
import gazebo_communicator.GazeboCommunicator as gc
from path_planning.Heightmap import Heightmap
from path_planning.Point import Point
#import task_management.TerritoryCleaning as tc
import task_management.Clusterization as cl
import path_planning.PathPlanner as pp
import path_planning.Constants as const
from path_planning.MovementManager import MovementManager
import numpy as np
from gazebo_communicator.Robot import Robot
from scipy.spatial.transform import Rotation
from gazebo_communicator.Deliverybot import Deliverybot

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
name1 = 'sim_p3at1'
name2 = 'sim_p3at2'
p1 = Point(-5, 0.5, 0.2)
p2 = Point(2, 0, 0.2)
g1 = Point(3, 0, 0)
g2 = Point(-6, 0, 0)

vect1 = p1.get_angle_between_points(g1)
rot1 = Rotation.from_euler('xyz', [0, 0, vect1], degrees=True)
quat1 = rot1.as_quat()

vect2 = p2.get_angle_between_points(g2)
rot2 = Rotation.from_euler('xyz', [0, 0, vect2], degrees=True)
quat2 = rot2.as_quat()

gc.spawn_worker(name1, p1, quat1)
gc.spawn_worker(name2, p2, quat2)
	
robot1 = Robot(name1)
robot2 = Robot(name2)
robots = {name1: robot1, name2: robot2}
mm = MovementManager(None, robots)
robot1.path = [g1]
robot2.path = [g2]
sleep(2)
mm.start()
print('Finish!')
rospy.spin()

