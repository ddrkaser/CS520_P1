#!/usr/bin/env python3

import numpy as np

# creating gridworld
def generate_gridworld(length,width,p):
    grid = np.random.choice([0,1],length*width, [1.0-p,p]).reshape(length,width)
    grid[0][0] = 0
    grid[-1][-1] = 0
    return grid

#A* Algorithm
def hureisticValue(point1, point2):
    x1,y1=point1
    x2,y2=point2
    return abs(x1-x2)+abs(y1-y2)
"""
g[currentCell] is the cost from start cell to currentCell
h[currentCell] is a heuristic cost from currentCell to the goal
f[currentCell]=g[currentCell]+h[currentCell]
"""

def algorithmA(grid,start,end, is_known):
    g={(x, y):float("inf") for y, eachRow in enumerate(grid) for x, eachcolumn in enumerate(eachRow)}
    g[start]=0
    f={(x, y):float("inf") for y, eachRow in enumerate(grid) for x, eachcolumn in enumerate(eachRow)}
    start = (0,0)
    #goal = (length,width)
    f[start]=hureisticValue(start, end)
    h = {(x, y): hureisticValue((x, y), end) for y, eachRow in enumerate(grid) for x, eachcolumn in enumerate(eachRow)}
    parent = {}
	
    pq=set([ (f[start], start) ]) # add start cell and distance information to the priority queue.
	
    curr_knowledge = [[0 for i in range(len(grid))] for j in range(len(grid[0]))]
    if is_known:
        curr_knowledge = grid
    print(curr_knowledge)
    while True:
        n = min(pq)
        pq.remove(n)
        successors = []
        curr_pos = n[1]
		
		# if goal node removed from priority queue, shortest path found
        if curr_pos == end:
            shortest_path = []
            path_pointer = end
            while path_pointer != start:
                shortest_path.append(path_pointer)
                path_pointer = parent[path_pointer]
            shortest_path = shortest_path[::-1]
            print(shortest_path)
            break
			
		# A* algorithm, based on assignment instructions
		# Determine which neighbors are valid successors
        if curr_pos[0] > 0 and curr_knowledge[curr_pos[1]][curr_pos[0] - 1] == 0: # the current node has a neighbor to its left which is unblocked
            left_neighbor = (curr_pos[0] - 1, curr_pos[1])
            if g[left_neighbor] > g[curr_pos] + 1: # if neighbor is undiscovered
                successors.append(left_neighbor)
				
        if curr_pos[0] < len(curr_knowledge[0])  - 1 and curr_knowledge[curr_pos[1]][curr_pos[0] + 1] == 0: # the current node has a neighbor to its right which is unblocked
            right_neighbor = (curr_pos[0] + 1, curr_pos[1])
            if g[right_neighbor] > g[curr_pos] + 1: # if neighbor is undiscovered
                successors.append(right_neighbor)
		
        if curr_pos[1] > 0 and curr_knowledge[curr_pos[1] - 1][curr_pos[0]] == 0: # the current node has a neighbor to its top which is unblocked
            top_neighbor = (curr_pos[0], curr_pos[1] - 1)
            if g[top_neighbor] > g[curr_pos] + 1: # if neighbor is undiscovered
                successors.append(top_neighbor)
				
        if curr_pos[1] < len(curr_knowledge) - 1 and curr_knowledge[curr_pos[1] + 1][curr_pos[0]] == 0: # the current node has a neighbor to its bottom which is unblocked
            bottom_neighbor = (curr_pos[0], curr_pos[1] + 1)
            if g[bottom_neighbor] > g[curr_pos] + 1: # if neighbor is undiscovered
                successors.append(bottom_neighbor)
				
        # Update shortest paths and parents for each valid successor and add to priority queue, per assignment instructions
        for successor in successors:
            g[successor] = g[curr_pos] + 1
            parent[successor] = curr_pos
            pq.add((g[successor] + h[successor], successor))
        
        if len(pq) == 0: # if priority queue is empty at any point, then unsolvable
            return False
       
	   # Move pointer square by square along path
			# If new square UNBLOCKED, update curr_knowledge. If BLOCKED, restart loop
		
algorithmA(generate_gridworld(5,5,.5), (0,0), (4,4), True)