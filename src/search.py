# Using A* algorithm searches graph made in graph_builder.py to find
# shortest path from starting position to ending position


def a_star(graph, start, goal):
	''' Algorithm ends with a dictionary mapping nodes to their parent nodes'''
	frontier = []
	frontier.append((tuple(start),0)) # frontier consists of nodes 
	came_from = {}
	current_cost = {}
	came_from[tuple(start)] = None
	current_cost[tuple(start)] = 0

	while len(frontier)!=0:
		current, value = frontier.pop(0)
		if current == goal:
			break
		#print('current :', current)
		for n in graph.neighbors[current]:
			#print('neighbor: ', n)
			new_cost = current_cost[current] + graph.cost(current,n)
			if n not in current_cost or new_cost<current_cost[n]:
				#print('here')
				current_cost[n] = new_cost
				priority = new_cost + graph.heuristic(n, goal)
				#print('f1', frontier)
				frontier = queue_insert(frontier, n, priority)
				#print('f2', frontier)
				came_from[n] = current

	return process_astar(came_from, tuple(start), tuple(goal))


def queue_insert(queue, node, weight):
	if len(queue) == 0:
		return [(node, weight)]
	else:
		for i in range(len(queue)):
			if queue[i][1]>weight:
				return queue[:i] + [(node,weight)] + queue[i:]
		else:
			return queue + [(node,weight)]

def process_astar(came_from, start, goal):
	points = []
	node = goal
	while node!=start:
		points.append(node)
		#print(node)
		node = came_from[node]
	points.append(start)
	return points[::-1]