# Import modules
import task_management.TaskConstants as const
from math import sqrt
import time
import numpy as np
import random
import copy
import io
from collections import deque
import multiprocessing as mp
import pyswarms as ps
import logging
from pyswarms.backend.swarms import Swarm
from pyswarms.backend.operators import compute_pbest, compute_objective_function
from pyswarms.backend.handlers import BoundaryHandler, VelocityHandler, OptionsHandler

from math import sqrt
import pyswarms.backend as P
from pyswarms.backend.topology import Star
from pyswarms.backend.topology.random import Random


class MyOptimizer(ps.single.GlobalBestPSO):

	def optimize(
		self, objective_func, iters, n_processes=None, verbose=True, **kwargs):
		if verbose:
			log_level = logging.INFO
		else:
			log_level = logging.NOTSET

		self.objective_func = objective_func
		self.rep.log("Obj. func. args: {}".format(kwargs), lvl=logging.DEBUG)
		self.rep.log(
			"Optimize for {} iters with {}".format(iters, self.options),
			lvl=log_level,
		)
		# Populate memory of the handlers
		self.bh.memory = self.swarm.position
		self.vh.memory = self.swarm.position

		# Setup Pool of processes for parallel evaluation
		pool = None if n_processes is None else mp.Pool(n_processes)

		self.swarm.pbest_cost = np.full(self.swarm_size[0], np.inf)
		self.last_pbest_mas = self.swarm.pbest_cost.copy()
		ftol_history = deque(maxlen=self.ftol_iter)
		self.cur_iter = 0
		for i in self.rep.pbar(iters, self.name) if verbose else range(iters):
			self.cur_iter += 1
			# Compute cost for current position and personal best
			# fmt: off
			self.swarm.current_cost = compute_objective_function(self.swarm, objective_func, pool=pool, **kwargs)
			self.swarm.pbest_pos, self.swarm.pbest_cost = compute_pbest(self.swarm)
			# Set best_cost_yet_found for ftol
			best_cost_yet_found = self.swarm.best_cost
			self.swarm.best_pos, self.swarm.best_cost = self.top.compute_gbest(self.swarm)
			print('best_cost: ' + str(self.swarm.best_cost))
			self.calc_lod_gbest()
			self.calc_lod_pbest()
			# fmt: on
			if verbose:
				self.rep.hook(best_cost=self.swarm.best_cost)
			# Save to history
			hist = self.ToHistory(
				best_cost=self.swarm.best_cost,
				mean_pbest_cost=np.mean(self.swarm.pbest_cost),
				mean_neighbor_cost=self.swarm.best_cost,
				position=self.swarm.position,
				velocity=self.swarm.velocity,
			)
			self._populate_history(hist)
			# Verify stop criteria based on the relative acceptable cost ftol
			relative_measure = self.ftol * (1 + np.abs(best_cost_yet_found))
			delta = (
				np.abs(self.swarm.best_cost - best_cost_yet_found)
				< relative_measure
			)
			if i < self.ftol_iter:
				ftol_history.append(delta)
			else:
				ftol_history.append(delta)
				if all(ftol_history) or self.LOD_g == 100:
					break
			# Perform options update
			self.swarm.options = self.oh(
				self.options, iternow=i, itermax=iters
			)
			# Perform velocity and position updates
			self.swarm.velocity = self.top.compute_velocity(
				self.swarm, self.velocity_clamp, self.vh, self.bounds
			)
			self.swarm.position = self.top.compute_position(
				self.swarm, self.bounds, self.bh
			)
		# Obtain the final best_cost and the final best_position
		final_best_cost = self.swarm.best_cost.copy()
		final_best_pos = self.swarm.pbest_pos[
			self.swarm.pbest_cost.argmin()
		].copy()
		# Write report in log and return final cost and position
		self.rep.log(
			"Optimization finished | best cost: {}, best pos: {}".format(
				final_best_cost, final_best_pos
			),
			lvl=log_level,
		)
		# Close Pool of Processes
		if n_processes is not None:
			pool.close()
		return (final_best_cost, final_best_pos)

	def calc_path_cost(self, part_pos):

		task_assignment = {i: int(part_pos[i]) for i in range(len(part_pos))}
		tmp_robots = copy.copy(self.robots)
		costs = {}

		for key in task_assignment.keys():

			r_ind = task_assignment[key]
			try:
				robot = tmp_robots[r_ind]
			except IndexError:
				return np.inf

			t_id = self.targets[key].id
			path_key = get_path_key(robot.name, t_id)
			path_cost = self.path_costs[path_key]
			if costs.get(r_ind):

				costs[r_ind] += path_cost

			else:

				costs[r_ind] = path_cost

		max_cost = max(costs.values())
		return max_cost

	def cost_func(self, x):

		total_costs = []

		for item in x:

			total_cost = self.calc_path_cost(item)
			total_costs.append(total_cost)

		cost_array = make_ndarray_line(total_costs, (len(total_costs),))
		return cost_array

	def calc_lod_gbest(self):

		self.gbest_hist.append((self.swarm.best_pos, self.swarm.best_cost))
		if len(self.gbest_hist) >= 2:

			cur_gbest = self.swarm.best_cost
			last_gbest = self.gbest_hist[-2][1]
			if cur_gbest >= last_gbest:

				self.LOD_g += 1

				if self.LOD_g >= self.Sg:

					cur_gbest_pos = self.swarm.best_pos
					if cur_gbest > self.pbest_temp[1]:

						self.swarm.best_pos = self.pbest_temp[0]
						self.swarm.best_cost = self.pbest_temp[1]

			else:

				self.LOD_g = 0


	def calc_lod_pbest(self):

		for ind in range(len(self.swarm.pbest_cost)):

			cur_pbest = self.swarm.pbest_cost[ind]
			cur_pbest_pos = self.swarm.pbest_pos[ind]
			last_pbest = self.last_pbest_mas[ind]

			if cur_pbest >= last_pbest:

				self.LOD_p[ind] += 1

				if self.LOD_p[ind] >= self.Sp:

					temp_cost = np.inf
					while temp_cost == np.inf:

						i1 = random.randint(0, self.swarm.n_particles - 1)
						i2 = random.randint(0, len(self.gbest_hist) - 1)
						new_pbest_pos = self.swarm.pbest_pos[i1] + (i1 + 1) / (i1 + i2 + 2) * (self.gbest_hist[i2][0] - cur_pbest_pos)
						temp_cost = self.calc_path_cost(new_pbest_pos)
						self.pbest_temp = (self.swarm.pbest_cost[ind].copy(), temp_cost)


			else:

				self.LOD_p[ind] = 0

		self.last_pbest_mas = self.swarm.pbest_cost.copy()


	def init_dd_attrs(self, robots, targets, path_costs):
		self.robots = robots
		self.targets = targets
		self.path_costs = path_costs
		self.Sp = const.S_P
		self.Sg = const.S_G
		self.LOD_g = 0
		self.LOD_p = [0 for i in range(self.swarm.n_particles)]
		self.gbest_hist = []

class DDPSO:

	def __init__(self, robots, targets, path_costs):
	
		if type(robots) == dict:
	
			self.robots = convert_to_mas(robots)
			
		else:
		
			self.robots = robots
			
		if type(targets) == dict:

			self.targets = convert_to_mas(targets)
			
		elif type(targets) == list:
		
			self.targets = targets
			
		else:
		
			self.targets = [targets]
			
		self.path_costs = path_costs

	def make_swarm(self, n_particles, dimensions, options):

		init_positions = ps.backend.generate_swarm(n_particles=n_particles, dimensions=dimensions)
		init_velocities = ps.backend.generate_velocity(n_particles=n_particles, dimensions=dimensions)
		#my_options = {'foo': 0.4, 'bar': 0.6}

		my_swarm = Swarm(position=init_positions, velocity=init_velocities, options=options)
		return my_swarm

	def run_pso(self):
		max_bound = len(self.robots) * np.ones(len(self.targets))
		min_bound = 0 * np.ones(len(self.targets))
		bounds = (min_bound, max_bound)
		my_options = {'c1': 0.6, 'c2': 0.3, 'w': 0.4}
		optimizer = MyOptimizer(n_particles=const.PARTICLES_NUM, dimensions=len(self.targets), options=my_options, bounds=bounds)
		s_time = time.time()
		optimizer.init_dd_attrs(self.robots, self.targets, self.path_costs)
		cost, pos = optimizer.optimize(optimizer.cost_func, const.PSO_ITER_COUNT)
		f_time = time.time()
		int_pos = [int(item) for item in pos]
		print('best_cost: ' + str(cost))
		print(int_pos)
		print('Optimization run time: ' + str(f_time - s_time))
		return int_pos

def make_ndarray_line(mas, shape):

	new_array = np.ndarray(shape)

	for i in range(shape[0]):

		new_array[i] = mas[i]

	return new_array

def make_ndarray(mas, shape):

	new_array = np.ndarray(shape)

	for i in range(shape[0]):

		for j in range(shape[1]):

			new_array[i][j] = mas[i][j]

	return new_array

def get_random_point(boundaries):

	min_b = boundaries[0]
	max_b = boundaries[1]
	x = random.uniform(min_b, max_b)
	y = random.uniform(min_b, max_b)
	z = random.uniform(min_b, max_b)
	p = Point(x, y, z)
	return p

def get_random_point_mas(count, boundaries):

	p_mas = [get_random_point(boundaries) for i in range(count)]
	return p_mas

def save_poses(target_poses, f_name):

	f = open(f_name, 'w+')

	for pos in target_poses:

		print(pos)
		f.write(str(pos) + '\n')

	f.close()


def get_poses_from_file(count, f_name):
	poses = []

	with io.open(f_name, encoding='utf-8') as file:

		for line in file:

			pos = line.split(' ')
			x = float(pos[0])
			y = float(pos[1])
			z = float(pos[2])
			p = Point(x, y, z)
			poses.append(p)
			if len(poses) == count:
				break

		return poses
		
def get_path_key(robot_key, target_key):

	path_key = str(robot_key) + '-' + str(target_key)
	return path_key

def convert_to_mas(init_dict):

	mas = []
	for key in init_dict.keys():
	
		mas.append(init_dict[key])
		
	return mas
