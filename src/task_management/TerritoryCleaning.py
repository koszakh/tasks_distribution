# The module for garbage distribution

import numpy as np
import task_management.Clusterization as cl
import task_management.DDPSO as pso_opt
from path_planning.PathPlanner import get_end_vect

class TerritoryCleaning:

	def __init__(self, robots, garbage_poses, mh):
	
		self.robots = {r_key: robots[r_key] for r_key in list(robots.keys()) if robots[r_key].manipulator}
		self.garbage_poses = garbage_poses
		self.mh = mh
		
	def distribute_garbage(self):

		paths, path_costs = self.mh.calc_paths(self.robots, self.garbage_poses)
		cc = cl.CustomClustering(self.robots, self.garbage_poses, self.mh, path_costs)
		clust_dict = cc.start_rev_clustering()
		g_paths = {}
		for r_key in clust_dict:

			g_paths[r_key] = []
			target_ids = clust_dict[r_key]
			robot = self.robots[r_key]
			r_pos = robot.get_robot_position()
			r_pos_key = self.mh.get_nearest_vertice_id(r_pos.x, r_pos.y)
			last_r_vect = robot.get_robot_orientation_vector()
			if len(clust_dict[r_key]) > 0:

				for t_key in target_ids:
				
					path, path_cost = self.mh.find_path(r_pos_key, t_key, last_r_vect)
					r_pos_key = t_key
					last_r_vect = get_end_vect(path)
					g_paths[r_key].append(path)
				
			else:

				print(str(r_key) + ' is isolated!')
				
		return g_paths


	
