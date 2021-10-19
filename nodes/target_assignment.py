#!/usr/bin/env python

# Path planning node for ground targets
import path_planning.TargetAssignment as ta
import gazebo_communicator.GazeboConstants as const
import gazebo_communicator.BatteryTracker as bt
from path_planning.MovementManager import MovementManager
from gazebo_communicator.Worker import Worker
from gazebo_communicator.Charger import Charger
from targets_path_planning.msg import AllPaths, WorkPath, Path, Poses, RobotState, Vector2d, NamesList
from geometry_msgs.msg import Point
import rospy
import time
import random
import copy
import gazebo_communicator.GazeboConstants as gc_const
from charge.charge import eval_distance
from energy_check import define_charging_points, charge_alloc, sort_by_distance
from mission_time.mission_time import eval_mission_time

def prepare_all_paths_msg(names, paths, workpoints):

	msg = AllPaths()
	msg.all_paths = []

	for name in names:

		path = paths[name]
		robot_workpoints = workpoints[name]
		path_msg = prepare_paths_msg(name, path, robot_workpoints)
		msg.all_paths.append(path_msg)

	return msg

def prepare_paths_msg(name, paths, workpoints):

	msg = WorkPath()
	msg.paths = []
	msg.workpoints = prepare_path_msg(workpoints)
	msg.robot_name = name

	for path in paths:

		path_msg = prepare_path_msg(path)
		msg.paths.append(path_msg)
		
	return msg

def prepare_path_msg(path):

	msg = Path()
	msg.path = []
	
	for state in path:
	
		point_msg = prepare_point_msg(state)
		msg.path.append(point_msg)
	
	return msg
	
def prepare_point_msg(point):

	#print('point: ' + str(point))
	msg = Point()
	msg.x = point.x
	msg.y = point.y
	msg.z = point.z
	
	return msg
	
def prepare_orient_msg(orient):

	vect = Vector2d()
	vect.x = orient.x
	vect.y = orient.y
	return vect

def prepare_poses_msg(poses, orients):

	msg = Poses()
	msg.robot_states = []
	
	for name in poses.keys():

		robot_msg = RobotState()
		robot_msg.robot_name = name
		robot_msg.pose = prepare_point_msg(poses[name])
		robot_msg.orient = prepare_orient_msg(orients[name])
		msg.robot_states.append(robot_msg)
		
	return msg

def prepare_names_msg(names):

	msg = NamesList()
	msg.names = []
	
	for name in names:
	
		msg.names.append(name)
		
	return msg

def get_random_els(mas, el_count):

	copy_mas = []

	for i in range(el_count):

		item = random.choice(mas)
		copy_mas.append(item)
		mas.remove(item)

	return copy_mas

def subtraction_of_set(mas1, mas2):

	new_mas = []

	for n in mas1:

		if not mas2.__contains__(n):

			new_mas.append(n)

	return new_mas

def calc_path_cost(path):
	path_cost = 0
	for i in range(1, len(path)):
		current_p = path[i]
		last_p = path[i - 1]
		distance = eval_distance(current_p, last_p)
		path_cost += distance*gc_const.MOVE_CHARGE_LOSS_COEF
	return path_cost


def define_new_dict_of_workpoints(workpoints, poses, orients, workpoints_was_deleted):
	copy_wpts = copy.copy(workpoints)
	for c_name in c_names:
		for w_name in workpoints.keys():
			wpts = copy_wpts[w_name]
			i = 0
			while i < len(wpts):
				workpoint = wpts[i]
				start_id = t_as.mh.get_nearest_vertice_id(poses[c_name].x, poses[c_name].y)
				goal_id = t_as.mh.get_nearest_vertice_id(workpoint.x, workpoint.y)

				path, path_ids = t_as.mh.find_path(start_id, goal_id, orients[c_name])

				path_cost = calc_path_cost(path) * 2
				rospy.loginfo("Path cost of " + c_name + " to workpoint of " + w_name + ": " + str(path_cost))
				if path_cost > gc_const.HIGH_LIMIT_BATTERY - gc_const.LOWER_LIMIT_BATTERY:
					rospy.logwarn("This workpoint has been deleted.")
					copy_wpts[w_name].pop(i)
					workpoints_was_deleted = True
					continue
				i += 1
	return copy_wpts, workpoints_was_deleted


def filter_dict_of_workpoints(workpoints):
	# Delete keys where len(workpoints) = 0
	new_dict = copy.copy(workpoints)
	for w_name in new_dict.keys():
		if len(new_dict[w_name]) == 0:
			del new_dict[w_name]
	return new_dict


def calculate_new_paths(copy_wpts):
	# Calculate new paths for reduced dict of workpoints
	new_paths = dict()
	for w_name in copy_wpts.keys():
		paths_w_name = t_as.calc_task_path(w_name, copy_wpts[w_name])
		new_paths[w_name] = paths_w_name
	return new_paths


def make_new_list_of_w_names(copy_wpts):
	new_w_names = []
	for w_name in copy_wpts.keys():
		new_w_names.append(w_name)
	return new_w_names


def init_paths_dict():
	paths_to_ch_p = dict()
	paths_to_base = dict()
	for c_name in robot_allocation.keys():
		paths_to_ch_p[c_name] = []
		paths_to_base[c_name] = []
	return paths_to_ch_p, paths_to_base


def fullfill_paths_dicts(paths_to_ch_p, paths_to_base):
	paths_of_ch_robots_to_ch_p = copy.copy(paths_to_ch_p)
	paths_of_ch_robots_to_base = copy.copy(paths_to_base)
	for c_name in robot_allocation.keys():
		start_id = t_as.mh.get_nearest_vertice_id(poses[c_name].x, poses[c_name].y)
		ch_pts = robot_allocation[c_name]
		for pt in ch_pts:
			
			pre_ch_pt = pt[0]
			ch_pt = pt[1]

			goal_id = t_as.mh.get_nearest_vertice_id(pre_ch_pt.x, pre_ch_pt.y)
			ch_path, path_ids = t_as.mh.find_path(start_id, goal_id, orients[c_name])
			paths_of_ch_robots_to_ch_p[c_name].append(ch_path)
			
			end_vect = ch_pt.last_vect
			last_id = ch_pt.id
			base_path, path_ids = t_as.mh.find_path(last_id, start_id, end_vect)
			paths_of_ch_robots_to_base[c_name].append(base_path)
	return paths_of_ch_robots_to_ch_p, paths_of_ch_robots_to_base

rospy.init_node('target_assignment')

paths_pub = rospy.Publisher('worker_paths', AllPaths, queue_size=10)
poses_pub = rospy.Publisher('robot_poses', Poses, queue_size=10)
c_names_pub = rospy.Publisher('chargers_names', NamesList, queue_size=10)

robots_count = const.WORKERS_COUNT + const.CHARGERS_COUNT

names = ['sim_p3at' + str(i) for i in range(1, robots_count + 1)]
w_names = get_random_els(names, const.WORKERS_COUNT)
c_names = subtraction_of_set(names, w_names)
old_w_names = w_names

t_as = ta.TargetAssignment(w_names, c_names, const.TARGETS_COUNT)
s_time = time.time()
paths, workpoints = t_as.target_assignment()
f_time = time.time()

b_w_names = paths.keys()

exec_time = f_time - s_time
print('Target assignment time: ' + str(exec_time))

poses, orients = t_as.get_robots_pos_orient()

workpoints_was_deleted = False
copy_wpts, workpoints_was_deleted = define_new_dict_of_workpoints(workpoints, poses, orients, workpoints_was_deleted)

if workpoints_was_deleted:
	copy_wpts = filter_dict_of_workpoints(copy_wpts)
	new_paths = calculate_new_paths(copy_wpts)
	new_w_names = make_new_list_of_w_names(copy_wpts)

	w_names = new_w_names
	paths = new_paths
	workpoints = copy_wpts
	
paths, flag = t_as.calc_task_paths(workpoints)

charging_points = define_charging_points(paths, workpoints)
# print("Charging points: " + str(charging_points) + "\n")

sorted_charging_points = sort_by_distance(charging_points)
# print("Sorted charging points: " + str(sorted_charging_points) + "\n")

robot_allocation = charge_alloc(sorted_charging_points, c_names)
# print("Robot allocation" + str(robot_allocation))

paths_to_ch_p, paths_to_base = init_paths_dict()
paths_of_ch_robots_to_ch_p, paths_of_ch_robots_to_base = fullfill_paths_dicts(paths_to_ch_p, paths_to_base)

mission_time = eval_mission_time(paths, paths_of_ch_robots_to_ch_p, paths_of_ch_robots_to_base, robot_allocation, charging_points, workpoints)

mm = MovementManager(t_as.mh, old_w_names, c_names)
mm.prepare_robots(paths, workpoints, charging_points, robot_allocation, paths_of_ch_robots_to_ch_p, paths_of_ch_robots_to_base)
mm.start()

#mm.prepare_robots()

#poses_msg = prepare_poses_msg(poses, orients)
#poses_pub.publish(poses_msg)

#names_msg = prepare_names_msg(c_names)
#c_names_pub.publish(names_msg)

#paths_msg = prepare_all_paths_msg(w_names, paths, workpoints)
#paths_pub.publish(paths_msg)
