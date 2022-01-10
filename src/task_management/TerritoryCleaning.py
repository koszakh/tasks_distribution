# The module for garbage distribution

#import PyGMO import *
import pyswarms as ps
from pyswarms.utils.functions import single_obj as fx
import numpy as np
#import gazebo_communicator.GazeboCommunicator as gc

def distribute_cleaning_robots(garbage_location, robot_names):

	robot_poses = [gc.get_model_position(name) for name in robot_names]
	distributed_garbage = distribute_garbage(robot_poses, garbage_location)
	return distributed_garbage
	
def distibute_garbage(robot_poses, garbage_location):

	distributed_garbage = {}
	return distributed_garbage
	
def pso_test():

	options = {'c1': 0.5, 'c2': 0.3, 'w':0.9}

	# Call instance of PSO
	optimizer = ps.single.GlobalBestPSO(n_particles=10, dimensions=2, options=options)

	# Perform optimization
	cost, pos = optimizer.optimize(fx.sphere, iters=1000)

	
