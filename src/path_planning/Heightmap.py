# Module that converts the height map into a view convenient for path planning

import rospy
import path_planning.Constants as const
import path_planning.PathPlanner as pp
from path_planning.Point import Point
import gazebo_communicator.GazeboCommunicator as gc
import gazebo_communicator.GazeboConstants as gc_const
import cv2
import io
import copy

def viewImage(image, name_of_window):

    cv2.namedWindow(name_of_window, cv2.WINDOW_NORMAL)
    cv2.imshow(name_of_window, image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

# Heightmap class used for generating cell matrix
# heightmap: heightmap vertex list
class Heightmap:

	def __init__(self, sdf_path):
	
		self.sdf_path = sdf_path
		self.png_path = self.get_png_path()
		self.image = cv2.imread(self.png_path, 0)
		#self.image_copy = cv2.imread(self.png_path, cv2.IMREAD_COLOR)
		#viewImage(self.image, 'Map')
		#print(self.image.shape[0], self.image.shape[1])
		self.length_scale, self.width_scale, self.height_scale = self.get_map_size()
		self.get_map_pos()
		#self.p_count = 0

	def get_map_size(self):
		
		with io.open(self.sdf_path, encoding='utf-8') as file:
			
			for word in const.MAP_SIZE_WORDS:
				
				for line in file:
					
					if word in line:
					
						break
						
		size = line[line.find('>') + 1:line.rfind('<')]
		s = size.split(' ')
		length_scale = float(s[0])
		width_scale = float(s[1])
		p_value_range = self.calc_p_range()
		height_scale = float(p_value_range / float(s[2]))
		#print(str(s[0]), str(s[1]), str(s[2]))
		return length_scale, width_scale, height_scale
		
	def get_png_path(self):
	
		with io.open(self.sdf_path, encoding='utf-8') as file:
		
			for word in const.MAP_PATH_WORDS:
			
				for line in file:
				
					if word in line:
					
						break
			
		path = line[line.find('/') + 1:line.rfind('<')]
		png_path = const.ROOT_PATH + path		
		return png_path
		
	def get_map_pos(self):
	
		with io.open(self.sdf_path, encoding='utf-8') as file:
			
			for word in const.MAP_POS_WORDS:
				
				for line in file:
					
					if word in line:
					
						break
						
		pos = line[line.find('>') + 1:line.rfind('<')]
		s = pos.split(' ')
		self.x = float(s[0])
		self.y = float(s[1])
		self.z = float(s[2])
		
	def calc_p_range(self):
	
		max_p_value = 0
		
		for i in range(self.image.shape[0]):
		
			for j in range(self.image.shape[1]):
				
				p_value = self.image[i][j]
					
				if p_value > max_p_value:
				
					max_p_value = p_value
					
		p_value_range = max_p_value - 0
		
		return p_value_range
		
	def calc_heightmap_bounds(self):
	
		self.min_col = const.COL_RANGE[0]
		self.max_col = const.COL_RANGE[1]
		self.min_row = const.ROW_RANGE[0]
		self.max_row = const.ROW_RANGE[1]
		
		if self.min_col < 0:
		
			self.min_col = 0
			
		elif self.max_col >= self.image.shape[0]:
		
			self.max_col = self.image.shape[0] - 1
			
		if self.min_row < 0:
		
			self.min_row = 0
			
		elif self.max_row >= self.image.shape[1]:
		
			self.max_row = self.image.shape[1] - 1
			
		rospy.set_param('min_col', self.min_col)
		rospy.set_param('max_col', self.max_col)
		rospy.set_param('min_row', self.min_row)
		rospy.set_param('max_row', self.max_row)

		#print('min_col: ' + str(self.min_col))
		#print('max_col: ' + str(self.max_col))
		#print('min_row: ' + str(self.min_row))
		#print('max_row: ' + str(self.max_row))
		
		self.x_grid_size = float(self.length_scale / (self.image.shape[0] - 1))
		self.y_grid_size = float(self.width_scale / (self.image.shape[1] - 1))
		self.steps_count = int(self.x_grid_size / const.DES_GRID_SIZE) + 1
		real_grid_size = float(self.x_grid_size / self.steps_count)
		rospy.set_param('real_grid_size', real_grid_size)
		print('Real grid size: ' + str(round(real_grid_size, 2)) + ' m')

	def get_all_heightmap_points(self):
	
		self.min_x = float('inf')
		self.max_x = -float('inf')
		self.min_y = float('inf')
		self.max_y = -float('inf')
		self.max_z = 0
		hmap = {(str(float(i)), str(float(j))): self.get_heightmap_point(i, j) for i in \
		 range(self.min_col, self.max_col + 1) for j in range(self.min_row, self.max_row + 1)}
		return hmap

	def get_heightmap_point(self, col, row):
	
		p_id = (str(float(col)), str(float(row)))
		x = float(-self.length_scale / 2 + row * self.x_grid_size) + self.x
		y = float(self.width_scale / 2 - col * self.y_grid_size) + self.y
		z = float(self.image[col][row] / self.height_scale) + self.z
			
		p = Point(x, y, z)
		p.set_id(p_id)
				
		if (col == self.min_col and row == self.min_row) or (col == self.max_col and row == self.min_row) or (col == self.min_col and row == self.max_row) or (col == self.max_col and row == self.max_row):
		
			pass
			gc.spawn_sdf_model(p, gc_const.BIG_RED_VERTICE_PATH, 'v' + str(p_id))
		
		return p

# Preparing a heightmap for further path planning (generation + converting to dictionary)
	def prepare_heightmap(self):
	
		self.calc_heightmap_bounds()
		hmap = self.get_all_heightmap_points()
		return hmap, self.length_scale, self.width_scale, self.x_grid_size, self.y_grid_size, self.steps_count

def prepare_map_handler():

	hm = Heightmap(const.HEIGHTMAP_SDF_PATH)
	hmap, l_scale, w_scale, x_step, y_step, step_count = hm.prepare_heightmap()
	mh = pp.PathPlanner(hmap, l_scale, w_scale, x_step, y_step, step_count)
	mh.gridmap_preparing()
	return mh
