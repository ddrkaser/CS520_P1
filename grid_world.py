#!/usr/bin/env python3

import numpy as np
from math import sqrt
import matplotlib.pyplot as plt 

# Generates a length * width size gridworld
# Each square has a probability chance of being blocked
# The start and end squares are set to unblocked
def generate_gridworld(length, width, probability):
    grid = np.random.choice([0,1], length * width, p = [1 - probability, probability]).reshape(length, width)
    grid[0][0] = 0
    grid[-1][-1] = 0
    return grid

# Constants to represent various types of heuristics
EUCLIDEAN_DIST = 0
MANHATTAN_DIST = 1
CHEBYSHEV_DIST = 2

# Calculates h(x) using one of a range of heuristics
def hureisticValue(point1, point2, heuristic_type = 1):
    x1,y1=point1
    x2,y2=point2
    if heuristic_type == 0:
        return sqrt((x1 - x2)**2 + (y1 - y2)**2)
    elif heuristic_type == 1:
        return abs(x1 - x2) + abs(y1 - y2)
    else:
        return max(abs(x1 - x2), abs(y1 - y2))
"""
g[currentCell] is the cost from start cell to currentCell
h[currentCell] is a heuristic cost from currentCell to the goal
f[currentCell]=g[currentCell]+h[currentCell]
"""

# Defines steps for running the A* algorithm once
def A_star(curr_knowledge, start, end, heuristic_type = 1):
    # Initializes the g(x), f(x), and h(x) values for all squares
    g = {(x, y):float("inf") for y, eachRow in enumerate(curr_knowledge) for x, eachcolumn in enumerate(eachRow)}
    g[start] = 0
    f = {(x, y):float("inf") for y, eachRow in enumerate(curr_knowledge) for x, eachcolumn in enumerate(eachRow)}
    f[start] = hureisticValue(start, end, heuristic_type)
    h = {(x, y): hureisticValue((x, y), end, heuristic_type) for y, eachRow in enumerate(curr_knowledge) for x, eachcolumn in enumerate(eachRow)}
    parent = {}
    visited={start} # it is a set which provide the uniqueness, means it is ensure that not a single cell visit more than onece.
    tiebreaker = 0
	# Creates a priority queue using a Python set, adding start cell and its distance information
    pq = set([ (f[start], tiebreaker, start) ])
    # A* algorithm, based on assignment instructions
    while True:
		# Remove the node in the priority queue with the smallest f value
        n = min(pq)
        print(n)
        pq.remove(n)
        successors = []
		# curr_pos is a tuple (x, y) where x represents the column the square is in, and y represents the row
        curr_pos = n[2]
        visited.remove(curr_pos)
		# if goal node removed from priority queue, shortest path found
        if curr_pos == end:
            shortest_path = []
            path_pointer = end
            while path_pointer != start:
                shortest_path.append(path_pointer)
                path_pointer = parent[path_pointer]
            shortest_path.append(start)
            shortest_path = shortest_path[::-1]
            print(shortest_path)
            return shortest_path
			
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
            if successor not in visited:
                tiebreaker += 1
                pq.add((g[successor] + h[successor], -tiebreaker, successor))
                visited.add(successor)
		# if priority queue is empty at any point, then unsolvable
        if len(pq) == 0:
            print("Not solvable")
            return False

# Handles processing of Repeated A*, restarting that algorithm if a blocked square is found in the determined shortest path
def algorithmA(grid, start, end, is_grid_known, has_four_way_vision):
    print(grid)
    # The assumed state of the gridworld at any point in time. For some questions, the current knowledge is unknown at the start
    curr_knowledge = [[0 for i in range(len(grid))] for j in range(len(grid[0]))]
    # If the grid is considered known to the robot, operate on that known grid
	# Else, the robot assumes a completely unblocked gridworld and will have to discover it as it moves
    if is_grid_known:
        curr_knowledge = grid
    print(curr_knowledge)
    
	# Run A* once on grid as known, returning False if unsolvable
    shortest_path = A_star(curr_knowledge, start, end)
    print(shortest_path)
    if not shortest_path:
        return False
    is_broken = False
    while True:
		# Move pointer square by square along path
        for sq in shortest_path:
            x = sq[0]
            y = sq[1]
			
			# If blocked, rerun A* and restart loop
            if grid[y][x] == 1:
                # If the robot can only see squares in its direction of movement, update its current knowledge of the grid to include this blocked square
                if not has_four_way_vision:
                    curr_knowledge[y][x] = 1				
                shortest_path = A_star(curr_knowledge, prev_sq, end)
                if not shortest_path:
                    return False
                is_broken = True
                break
			# If new square unblocked, update curr_knowledge. Loop will restart and move to next square on presumed shortest path
            else:
                 # If the robot can see in all compass directions, update squares adjacent to its current position
                 if has_four_way_vision:
                     if x != 0:
                         curr_knowledge[y][x - 1] = grid[y][x - 1]
                     if x < len(curr_knowledge[0]) - 1:
                         curr_knowledge[y][x + 1] = grid[y][x + 1]
                     if y != 0:
                         curr_knowledge[y - 1][x] = grid[y - 1][x]
                     if y < len(curr_knowledge) - 1:
                         curr_knowledge[y + 1][x] = grid[y + 1][x]
            prev_sq = sq
        if not is_broken:
            break
        is_broken = False
    return shortest_path

def plot_data_5():
    probability = []
    length = []
    for pr in range(0,10):
           density = np.random.choice([0,1])
           probability.append(density)
           length.append(random())#placholder value until we implement way to track path
 

def solvability_plot_4():
    prob = []
    solve = []
    for trial in range(0,101):
        probability = np.random.choice([0,1])
        solve.append(algorithmA(generate_gridworld(101,101,probability), (0,0), (101,101),True))
        prob.append(probability)
    
    plt.bar(prob,length,color="green")
    plt.xlabel("Probability")
    plt.ylabel("Solvability")
    plt.title("Solvability vs Probability")
    
    
solvability_plot_4()    
#plot_data_5()    



# Test algorithm
# is_grid_known is True for Questions 4, 5. It is False for later questions
# has_four_way_vision is True for all questions except 7
shortest_path = algorithmA(generate_gridworld(5, 5, 0.3), (0,0), (4,4), False, True)
#test1
grid = np.array([[0, 0, 0, 1],
       [0, 0, 1, 0],
       [0, 1, 0, 0],
       [0, 0, 0, 0]])
start = (0,0)
end = (len(grid)-1,len(grid)-1)
path_finder = A_star(grid, start, end)
