import task_management.Clusterization as cl
import task_management.DDPSO as pso_opt
import task_management.TaskConstants as const
import gazebo_communicator.GazeboCommunicator as gc
import gazebo_communicator.GazeboConstants as gc_const

class NetworkCreation:

	def __init__(self, robots, mh, beacon_pos, group_pos, group_route):

		self.robots = robots
		self.mh = mh
		self.beacon_pos = beacon_pos
		self.init_group_pos = group_pos
		self.group_route = group_route
		
	def get_current_robots_mas(self, names_mas):
	
		r_mas = []
		for name in names_mas:
		
			robot = self.robots[name]
			r_mas.append(robot)
			
		return r_mas

	def calc_key_poses(self):

		net_poses = [self.beacon_pos]
		last_net_p = self.beacon_pos
		gr_pos_dist = last_net_p.get_distance_to(self.init_group_pos)
		while gr_pos_dist > const.MAX_NET_DIST:
		
			new_p = last_net_p.get_point_at_distance_and_angle(self.init_group_pos, const.NODE_DIST)
			new_id = self.mh.get_nearest_vertice_id(new_p.x, new_p.y)
			new_net_p = self.mh.heightmap[new_id]
			net_poses.append(new_net_p)
			last_net_p = new_net_p
			gr_pos_dist = last_net_p.get_distance_to(self.init_group_pos)
		
		last_group_pos = self.init_group_pos
		ind = 0
		while not self.check_network_coverage(net_poses) and ind < len(self.group_route):
			
			next_group_pos, ind = self.get_next_group_pos(last_group_pos, ind)
			#new_p = last_net_p.get_point_at_distance_and_angle(next_group_pos, const.NODE_DIST)
			#new_id = self.mh.get_nearest_vertice_id(new_p.x, new_p.y)
			#new_net_p = self.mh.heightmap[new_id]
			last_group_pos = next_group_pos
			net_poses.append(next_group_pos)
		
		net_poses.pop(0)
		#print('\n > Nodes count: ' + str(len(net_poses)))
		net_poses_dict = {p.id: p for p in net_poses}
		#print(net_poses_dict.keys())
		return net_poses_dict
		
	def check_network_coverage(self, node_poses):
	
		coverage = True
		for p in self.group_route:
		
			dist = calc_min_dist_to_node(p, node_poses)
			if dist > const.MAX_NET_DIST:

				return False

		return coverage

	def create_network(self):

		key_poses = self.calc_key_poses()
		paths, path_costs = self.mh.calc_paths(self.robots, key_poses)
		cc = cl.CustomClustering(self.robots, key_poses, self.mh, path_costs)
		clust_dict = cc.start_clustering()
		net_dict = {}
		for t_key in clust_dict:

			#print('\n\n  Current robots count: ' + str(len(clust_dict[t_key])) + '\n\n')
			if len(clust_dict[t_key]) > 0:

				current_robots = self.get_current_robots_mas(clust_dict[t_key])
				pso_obj = pso_opt.DDPSO(current_robots, self.mh.heightmap[t_key], path_costs)
				solution = pso_obj.run_pso()
				#print('\n\n >>> Solution: ' + str(solution))
				r_name = current_robots[solution[0]].name
				net_dict[r_name] = t_key
				
			else:

				return {}

		nodes = [self.mh.heightmap[item] for item in list(net_dict.values())]
		#gc.visualise_path(nodes, gc_const.BIG_GREEN_VERTICE_PATH, 'node')
		node_paths = {}
		for r_key in list(net_dict.keys()):

			robot = self.robots[r_key]
			t_key = net_dict[r_key]
			r_pos = robot.get_robot_position()
			r_pos_id = self.mh.get_nearest_vertice_id(r_pos.x, r_pos.y)
			r_vect = robot.get_robot_orientation_vector()
			node_paths[r_key], path_cost = self.mh.find_path(r_pos_id, t_key, r_vect)

		return node_paths
		
	def get_next_group_pos(self, last_gr_pos, index):

		last_p = last_gr_pos
		for i in range(index, len(self.group_route)):
		
			next_p = self.group_route[i]
			dist = next_p.get_distance_to(last_gr_pos)
			if dist > const.NODE_DIST:

				return last_p, i
				
			last_p = next_p

		return last_p, i
			
def calc_min_dist_to_node(p, node_poses):

	min_dist = float('inf')
	
	for n_p in node_poses:
	
		dist = n_p.get_distance_to(p)

		if dist < min_dist:
		
			min_dist = dist
			
	return min_dist
