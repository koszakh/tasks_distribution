import numpy as np
from pyswarms.backend.swarms import Swarm
import random
import copy
import io

# Import sphere function as objective function
from pyswarms.utils.functions.single_obj import sphere as f

# Import backend modules
from math import sqrt
import pyswarms.backend as P
from pyswarms.backend.topology import Star
from pyswarms.backend.topology.random import Random
import pyswarms as ps

class PSO:

    def __init__(self, robots, targets):
        self.robots = robots
        self.targets = targets

    def make_swarm(self, n_particles, dimensions, options):

        #init_positions = make_init_positions(particle_poses, dimensions)
        #init_velocities = make_init_velocities()
        init_positions = ps.backend.generate_swarm(n_particles=n_particles, dimensions=dimensions)
        init_velocities = ps.backend.generate_velocity(n_particles=n_particles, dimensions=dimensions)
        #my_options = {'foo': 0.4, 'bar': 0.6}

        my_swarm = Swarm(position=init_positions, velocity=init_velocities, options=options)
        return my_swarm

    def run_pso(self, n_particles, cost_func):
        my_topology = Random()  # The Topology Class
        max_bound = len(self.robots) * np.ones(len(targets))
        min_bound = 0 * np.ones(len(targets))
        bounds = (min_bound, max_bound)
        my_options = {'c1': 0.6, 'c2': 0.3, 'w': 0.4}  # arbitrarily set
        my_swarm = self.make_swarm(n_particles, len(targets), my_options)#P.create_swarm(n_particles=50, dimensions=2, options=my_options)  # The Swarm Class

        print('The following are the attributes of our swarm: {}'.format(my_swarm.__dict__.keys()))
        optimizer = ps.single.GlobalBestPSO(n_particles=n_particles, dimensions=len(targets), options=my_options, bounds=bounds)
        # Perform optimization
        cost, pos = optimizer.optimize(cost_func, iters=5000)
        int_pos = [int(item) for item in pos]
        ta = {i: int(pos[i]) for i in range(len(pos))}
        t_cost = self.calc_path_cost(ta)
        print('t_cost: ' + str(t_cost))
        print(int_pos)
        #self.start_pso(my_swarm, my_topology)

    def calc_path_cost(self, task_assignment):

        tmp_robots = copy.copy(self.robots)
        costs = {}

        for key in task_assignment.keys():

            r_ind = task_assignment[key]
            #print('r_ind: ' + str(r_ind) + ' , ' + str(type(r_ind)))
            r_pos = tmp_robots[r_ind]
            t_pos = self.targets[key]
            dist = r_pos.get_distance_to(t_pos)
            tmp_robots[r_ind] = t_pos
            if costs.get(r_ind):

                costs[r_ind] += dist

            else:

                costs[r_ind] = dist

        max_cost = max(costs.values())
        return max_cost

    def cost_func(self, x):

        #targets = [(0, 0, 0), (10, 4, 6), (9, 9, 5)]
        total_costs = []
        for item in x:

            task_assignment = {i: int(item[i]) for i in range(len(item))}
            cur_robots = [int(r) for r in item]
            #print('particle: ' + str(cur_robots))
            #print(task_assignment)
            total_cost = self.calc_path_cost(task_assignment)
            total_costs.append(total_cost)

        cost_array = make_ndarray_line(total_costs, (len(total_costs),))
        #print(cost_array)
        return cost_array

    def start_pso(self, my_swarm, my_topology):

        iterations = 100  # Set 100 iterations
        for i in range(iterations):
            # Part 1: Update personal best
            my_swarm.current_cost = self.cost_func(my_swarm.position)  # Compute current cost
            my_swarm.pbest_cost = self.cost_func(my_swarm.pbest_pos)  # Compute personal best pos
            my_swarm.pbest_pos, my_swarm.pbest_cost = P.compute_pbest(my_swarm)  # Update and store

            # Part 2: Update global best
            # Note that gbest computation is dependent on your topology
            if np.min(my_swarm.pbest_cost) < my_swarm.best_cost:
                my_swarm.best_pos, my_swarm.best_cost = my_topology.compute_gbest(my_swarm, 2)

            # Let's print our output
            if i % 20 == 0:
                print('Iteration: {} | my_swarm.best_cost: {:.4f}'.format(i + 1, my_swarm.best_cost))

            # Part 3: Update position and velocity matrices
            # Note that position and velocity updates are dependent on your topology
            my_swarm.velocity = my_topology.compute_velocity(my_swarm)
            my_swarm.position = my_topology.compute_position(my_swarm)
            # print(my_swarm.velocity)

        # for pose in my_swarm.position:
        # print(pose)
        print('The best cost found by our swarm is: {:.4f}'.format(my_swarm.best_cost))
        print('The best position found by our swarm is: {}'.format(my_swarm.best_pos))

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
