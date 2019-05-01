import numpy as np
try:
	import cPickle as pickle
except:
	import pickle

class Graph(object):
	def __init__(self):
		self.nodes = set()
		self.neighbors = {} # dict of node mapped to edges
		self.x_max = None
		self.y_max = None
		self.resolution = None
		self.origin = tuple()
		self.map_resolution = '0.1m'
		#self.map_resolution = 0.5

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
		self.nodes.remove(node)
		for n in self.neighbors[node]:
		    self.neighbors[n].discard(node)
		del self.neighbors[node]

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

	def build_map(self, map, name, resolution, origin, from_file=False):
		fn = str(name)+str(self.map_resolution)+'.p'
		self.resolution = resolution
		self.origin = origin
		self.x_max = map.shape[0]-1
		self.y_max = map.shape[1]-1

		if from_file:
			try: 
				with open(fn, 'rb') as fp:
					#print 'loading graph from ' + fn
					data = pickle.load(fp)
					self.neighbors = data
					self.nodes = set(self.neighbors.keys())
				return
			except:
				print('failed to load map from file, generating and saving from scratch')

		for x in range(0, self.x_max+1, 1):
			for y in range(0, self.y_max+1, 1):
				if map[x,y] == 0:
					pos = (x,y)
					rwpose = self.occ_to_real_world(pos)
					self.add_node(rwpose)
					for coord in self.get_neighbor_coords(pos):
						x_prime = coord[0]
						y_prime = coord[1]
						if 0 <= x_prime <= self.x_max and 0 <= y_prime <= self.y_max:
							if map[x_prime, y_prime] == 0:
								rwcoord = self.occ_to_real_world(coord)
								self.add_node(rwcoord)
								self.add_edge(rwpose, rwcoord)
		#print 'saving graph to ' + fn

		with open(fn, 'wb') as fp:
			pickle.dump(self.neighbors, fp, protocol=pickle.HIGHEST_PROTOCOL)
		return

	def load_map(self, fn):
		try: 
			with open(fn, 'rb') as fp:
				#print 'loading graph from ' + fn
				data = pickle.load(fp)
				self.neighbors = data
				self.nodes = set(self.neighbors.keys())
			return
		except:
			print('failed to load map from file, make sure file name is correct and file located in ~/.ros')

	def heuristic(self, node1, node2):
		point1 = node1
		point2 = node2
		return ((point1[0]-point2[0])**2+(point1[1]-point2[1])**2)**(0.5)