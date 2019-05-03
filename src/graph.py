import numpy as np
try:
	import cPickle as pickle
except:
	import pickle

class Graph(object):
	def __init__(self):
		self.nodes = set()
		self.neighbors = {} # dict of node mapped to edges
		self.x_min = 0
		self.y_min = 0
		self.x_max = None
		self.y_max = None
		self.resolution = None
		self.origin = tuple()
		self.fn_resolution = '0.1m'
		self.fn = None
		self.params_fn = None

	def cost(self, p1, p2):
		return ((p1[0]-p2[0])**2+(p1[1]-p2[1])**2)**(0.5)

	def add_node(self, node):
		if node in self.nodes:
			return
		self.nodes.add(node)
		self.neighbors[node] = set()

	def add_edge(self, node1, node2):
		self.neighbors[node1].add(node2)

	def remove_node(self, node):
		#robust if a node isn't found in the graph
		self.nodes.discard(node)
		for n in self.neighbors[node]:
		    self.neighbors[n].discard(node)
		res = self.neighbors.pop(node, None)
		return res

	def __str__(self):
		for node in self.neighbors:
			print(node, ' : ', self.neighbors[node])
		return str(len(self.nodes))

	def get_neighbor_coords(self,coord):
		x = coord[0]
		y = coord[1]

		neighbors = {(x-1, y-1), (x-1, y), (x-1, y+1),
					(x, y-1),(x, y+1),
					(x+1, y-1),(x+1, y),(x+1, y+1)}

		return neighbors

	def occ_to_real_world(self, coord):
		#x = round((-coord[0]*self.resolution + self.origin[0])*2, 0)/2.
		x = round((-coord[0]*self.resolution + self.origin[0]), 1)
		#y = round((-coord[1]*self.resolution + self.origin[1])*2, 0)/2.
		y = round((-coord[1]*self.resolution + self.origin[1]), 1)
		return (x,y)

	def build_map(self, map, name, resolution, origin, x_bounds=None, y_bounds=None, save_params=False):
		self.fn = str(name)+ '_' + str(self.fn_resolution)+'.p'
		self.resolution = resolution
		self.origin = origin

		if x_bounds != None:
			self.x_min = x_bounds[0]
			self.x_max = x_bounds[1]
		else: self.x_max = map.shape[0]-1
		if y_bounds != None:
			self.y_min = y_bounds[0]
			self.y_max = y_bounds[1]
		else: self.y_max = map.shape[1]-1

		for x in range(self.x_min, self.x_max+1, 1):
			for y in range(self.y_min, self.y_max+1, 1):
				if map[x,y] == 0:
					pos = (x,y)
					rwpose = self.occ_to_real_world(pos)
					self.add_node(rwpose)
					for coord in self.get_neighbor_coords(pos):
						x_prime = coord[0]
						y_prime = coord[1]
						if self.x_min <= x_prime <= self.x_max and self.y_min <= y_prime <= self.y_max:
							if map[x_prime, y_prime] == 0:
								rwcoord = self.occ_to_real_world(coord)
								self.add_node(rwcoord)
								self.add_edge(rwpose, rwcoord)
		
		print('saving graph to ' + self.fn)
		with open(self.fn, 'wb') as fp:
			pickle.dump(self.neighbors, fp, protocol=pickle.HIGHEST_PROTOCOL)
		if save_params: self.save_params()
		return self.fn

	def save_params(self):
		self.params_fn = self.fn[:-2:] + '_params.p'
		output = {
			'fn': self.fn,
			'resolution': self.resolution,
			'origin': self.origin,
			'x_bounds': (self.x_min, self.x_max),
			'y_bounds': (self.y_min, self.y_max)
		}

		print('saving params to' + self.params_fn)
		with open(self.params_fn, 'wb') as fp:
			pickle.dump(output, fp, protocol=pickle.HIGHEST_PROTOCOL)
		return self.params_fn

	def load_map(self, fn, params_fn=None):
		try: 
			with open(fn, 'rb') as fp:
				print('loading graph from ' + fn)
				data = pickle.load(fp)
				self.neighbors = data
				self.nodes = set(self.neighbors.keys())
		except:
			print('failed to load map from file, make sure file name is correct and file located in ~/.ros')

		if params_fn != None:
			try:
				print('loading params from ' + params_fn)
				data = pickle.load(fp)
				self.fn = data['fn']
				self.resolution = data['resolution']
				self.origin = data['origin']
				self.x_min, self.x_max = data['x_bounds']
				self.y_min, self.y_max = data['y_bounds']
			except:
				print('failed to load params from file, make sure file name is correct and file located in ~/.ros')

	def heuristic(self, node1, node2):
		point1 = node1
		point2 = node2
		return ((point1[0]-point2[0])**2+(point1[1]-point2[1])**2)**(0.5)