import gazebo_communicator.GazeboCommunicator as gc
import gazebo_communicator.GazeboConstants as gc_const
import random
import copy
from task_management.DDPSO import get_path_key
import task_management.TaskConstants as const

class CustomClustering:

	def __init__(self, robots, targets, mh, path_costs):
		self.robots = robots
		self.targets = targets
		self.mh = mh
		self.path_costs = path_costs
		for t_id in self.targets.keys():

			t = self.targets[t_id]
			gc.spawn_sdf_model(t, gc_const.BIG_GREEN_VERTICE_PATH, 'target-' + str(t_id))



	def calc_total_cost(self, clust_dict):

		total_cost = 0
		for target_key in clust_dict.keys():

			clust_mas = clust_dict[target_key]
			for robot_key in clust_mas:
				path_key = get_path_key(robot_key, target_key)
				path_cost = self.path_costs[path_key]
				total_cost += path_cost

		return total_cost

	def start_clustering(self):
	
		#print('\nClusterization process started!\n')
		best_max_cost = float('inf')
		iter_num = 0
		change_counter = 0
		
		while iter_num < const.CLUSTERING_ITER_COUNT and change_counter < const.UNCHANGED_SEQ_LEN:

			iter_num += 1
			#print('\nIteration number: ' + str(iter_num) + ' | No change counter: ' + str(change_counter))
			clust_dict, max_cost = self.cluster_generation()
			#print('  Max cost: ' + str(max_cost))
			#print('  Best cost: ' + str(best_max_cost))
			if max_cost < best_max_cost:
			
				best_max_cost = max_cost
				best_clust_dict = clust_dict
				change_counter = 0
				
			else:
			
				change_counter += 1

		for key in best_clust_dict:

			#print('\nTarget id: ' + str(key))
			cluster = best_clust_dict[key]

			if len(cluster) > 0:

				for item in cluster:

					path_key = get_path_key(item, key)
					
			else:
			
				print('Task located at ' + str(key) + ' is isolated.')

		return best_clust_dict
		
	def cluster_generation(self):
	
		clust_dict = {}
		cur_targets_keys = copy.copy(list(self.targets.keys()))
		cur_robots_keys = copy.copy(list(self.robots.keys()))
		random.shuffle(cur_targets_keys)
		random.shuffle(cur_robots_keys)
		per_clust_count = len(self.robots) / len(self.targets) + 1
		
		for target_key in cur_targets_keys:
		
			clust_dict[target_key] = []
			
		for robot_key in cur_robots_keys:
		
			min_cost = float('inf')
			cur_target_key = None
			
			for target_key in cur_targets_keys:
			
				path_key = get_path_key(robot_key, target_key)
				path_cost = self.path_costs[path_key]
				
				if path_cost < min_cost and len(clust_dict[target_key]) < per_clust_count:
				
					min_cost = path_cost
					cur_target_key = target_key
					
			#print('Cur target key: ' + str(cur_target_key) + '\n')
			
			if cur_target_key:
			
				clust_dict[cur_target_key].append(robot_key)
				
		max_path_cost = self.calc_max_path_cost(clust_dict)
				
		return clust_dict, max_path_cost
		
	def calc_max_path_cost(self, clust_dict):
	
		max_path_cost = 0
		
		for key in clust_dict.keys():
		
			assigned_robots = clust_dict[key]
			
			for robot_key in assigned_robots:
			
				path_key = get_path_key(robot_key, key)
				path_cost = self.path_costs[path_key]
				
				if path_cost > max_path_cost:
				
					max_path_cost = path_cost
					
		return max_path_cost
		
	def start_rev_clustering(self):
	
		#print('\nClusterization process started!\n')
		best_max_cost = float('inf')
		iter_num = 0
		change_counter = 0
		
		while iter_num < const.CLUSTERING_ITER_COUNT and change_counter < const.UNCHANGED_SEQ_LEN:

			iter_num += 1
			#print('\nIteration number: ' + str(iter_num) + ' | No change counter: ' + str(change_counter))
			clust_dict, max_cost = self.rev_cluster_generation()
			#print('  Max cost: ' + str(max_cost))
			#print('  Best cost: ' + str(best_max_cost))
			if max_cost < best_max_cost:
			
				best_max_cost = max_cost
				best_clust_dict = clust_dict
				change_counter = 0
				
			else:
			
				change_counter += 1

		return best_clust_dict
		
	def rev_cluster_generation(self):
	
		clust_dict = {}
		cur_targets_keys = copy.copy(list(self.targets.keys()))
		cur_robots_keys = copy.copy(list(self.robots.keys()))
		random.shuffle(cur_targets_keys)
		random.shuffle(cur_robots_keys)
		per_clust_count = len(self.targets) / len(self.robots) + 1
		
		for robot_key in cur_robots_keys:
		
			clust_dict[robot_key] = []
			
		for target_key in cur_targets_keys:
		
			min_cost = float('inf')
			cur_robot_key = None
			
			for robot_key in cur_robots_keys:
			
				path_key = get_path_key(robot_key, target_key)
				path_cost = self.path_costs[path_key]
				
				if path_cost < min_cost and len(clust_dict[robot_key]) < per_clust_count:
				
					min_cost = path_cost
					cur_robot_key = robot_key
			
			if cur_robot_key:
			
				clust_dict[cur_robot_key].append(robot_key)
				
		max_path_cost = self.calc_rev_max_path_cost(clust_dict)
				
		return clust_dict, max_path_cost
		
	def calc_rev_max_path_cost(self, clust_dict):
	
		max_path_cost = 0
		
		for robot_key in clust_dict.keys():
		
			assigned_targets = clust_dict[robot_key]
			robot = self.robots[robot_key]
			r_pos = robot.get_robot_position()
			r_pos_key = self.mh.get_nearest_vertice_id(r_pos.x, r_pos.y)
			r_last_vect = robot.get_robot_orientation_vector()
			total_path_cost = 0
			
			for target_key in assigned_targets:
			
				#path_key = get_path_key(robot_key, target_key)
				#path_cost = self.path_costs[path_key]
				path, path_cost = self.mh.find_path(r_pos_key, target_key, r_last_vect)
				total_path_cost += path_cost
				r_last_vect = get_end_vect(path)

			if total_path_cost > max_path_cost:
			
				max_path_cost = total_path_cost
					
		return max_path_cost
