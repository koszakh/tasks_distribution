#!/usr/bin/env python3

import rospy
from time import sleep
import sys
#import gazebo_communicator.GazeboCommunicator as gc
import task_management.TerritoryCleaning as tc

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
print(sys.version)
tc.pso_test()


print('Finish!')
rospy.spin()

