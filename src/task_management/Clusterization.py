import gazebo_communicator.GazeboCommunicator as gc
import gazebo_communicator.GazeboConstants as gc_const
import random
import copy
from task_management.DDPSO import get_path_key
import task_management.TaskConstants as const

class CustomClustering:

	def __init__(self, robots, targets, path_costs):
		self.robots = robots
		self.targets = targets
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
	
		print('\nClusterization process started!\n')
		best_max_cost = float('inf')
		iter_num = 0
		change_counter = 0
		
		while iter_num < const.CLUSTERING_ITER_COUNT and change_counter < const.UNCHANGED_SEQ_LEN:

			iter_num += 1
			print('\nIteration number: ' + str(iter_num) + ' | No change counter: ' + str(change_counter))
			clust_dict, max_cost = self.cluster_generation()
			print('  Max cost: ' + str(max_cost))
			print('  Best cost: ' + str(best_max_cost))
			if max_cost < best_max_cost:
			
				best_max_cost = max_cost
				best_clust_dict = clust_dict
				change_counter = 0
				
			else:
			
				change_counter += 1

		for key in best_clust_dict:

			print('\nTarget id: ' + str(key))
			cluster = best_clust_dict[key]

			if len(cluster) > 0:

				for item in cluster:

					path_key = get_path_key(item, key)
					print(item, self.path_costs[path_key])
					
			else:
			
				print('Task located at ' + str(key) + ' is isolated.')

		return clust_dict

	def get_robot_min_path_cost(self, target_key):
		
		min_cost = float('inf')
		cur_robot_key = None
		
		for robot_key in self.robots.keys():
		
			path_key = get_path_key(robot_key, target_key)
			path_cost = self.path_costs[path_key]
		
			if path_cost < min_cost:
		
				min_cost = path_cost
				cur_robot_key = robot_key

		return min_cost, cur_robot_key
		
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
				#print(path_cost, target_key)
				
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
		
	def old_test(self):

		while True:
			
			clust_dict = {}
			total_cost = 0
			iter_num += 1
			
			for target_key in self.targets.keys():
				
				min_cost, cur_robot_key = self.get_robot_min_path_cost(target_key)
				total_cost += min_cost
				
				if clust_dict.get(target_key):
					
					clust_dict[target_key].append(cur_robot_key)
				
				else:
				
					clust_dict[target_key] = [cur_robot_key]

			if total_cost < best_cost:

				best_cost = total_cost
				best_clust_dict = clust_dict
			
			else:
			
				break
					
			print('\nTotal cost: ' + str(total_cost))
			print('Best cost: ' + str(best_cost) + '\n')
