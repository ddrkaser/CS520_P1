#!/usr/bin/env python3

import numpy as np
import pandas as pd
from math import sqrt
import matplotlib.pyplot as plt 
import time
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
    #count cell being processed
    cell_count = 0
    # A* algorithm, based on assignment instructions
    while not len(pq) == 0:
		# Remove the node in the priority queue with the smallest f value
        n = min(pq)
        pq.remove(n)
        cell_count += 1
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
            return [shortest_path, cell_count]
			
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
            return False

# Handles processing of Repeated A*, restarting that algorithm if a blocked square is found in the determined shortest path
def algorithmA(grid, start, end, is_grid_known, has_four_way_vision, heuristic_type = 1):
    # The assumed state of the gridworld at any point in time. For some questions, the current knowledge is unknown at the start
    curr_knowledge = [[0 for i in range(len(grid))] for j in range(len(grid[0]))]
    # If the grid is considered known to the robot, operate on that known grid
	# Else, the robot assumes a completely unblocked gridworld and will have to discover it as it moves
    if is_grid_known:
        curr_knowledge = grid
    complete_path = []
	# Run A* once on grid as known, returning False if unsolvable
    shortest_path = A_star(curr_knowledge, start, end)
    if not shortest_path:
        return False
    is_broken = False
    cell_count = shortest_path[1]
    while True:
		# Move pointer square by square along path
        for sq in shortest_path[0]:
            x = sq[0]
            y = sq[1]
			# If blocked, rerun A* and restart loop
            if grid[y][x] == 1:
                # If the robot can only see squares in its direction of movement, update its current knowledge of the grid to include this blocked square
                if not has_four_way_vision:
                    curr_knowledge[y][x] = 1	
                complete_path.remove(prev_sq)
                shortest_path = A_star(curr_knowledge, prev_sq, end,heuristic_type)                
                if not shortest_path:
                    return False
                is_broken = True
                cell_count += shortest_path[1]
                break
			# If new square unblocked, update curr_knowledge. Loop will restart and move to next square on presumed shortest path
            else:
                complete_path.append(sq)
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
    return [complete_path, cell_count, curr_knowledge]


def plot_data_6():
    probability = []
    length = []
    for pr in range(0,10):
           density = Random.random()
           probability.append(density)
           length.append(path_finder)
    
    plt.plot(probability,length,color="green")
    plt.xlabel("Probability")
    plt.ylabel("Path Length")
    plt.title("SPath Length vs Probability")
    
def plot_data_5():
    euclidean_vals = []
    manhattan_vals = []
    chebyshev_vals = []
    
    for heuristic in (EUCLIDEAN_DIST, MANHATTAN_DIST, CHEBYSHEV_DIST):
        trials = 0
        while trials < 100:
            t1 = time.time()
            res = algorithmA(generate_gridworld(101, 101, 0.3), (0, 0), (100, 100), True, True, heuristic)
            t2 = time.time()
            if not res:
                continue
            runtime = t2 - t1
            if heuristic == EUCLIDEAN_DIST:
                euclidean_vals.append(runtime)
            elif heuristic == MANHATTAN_DIST:
                manhattan_vals.append(runtime)
            else:
                chebyshev_vals.append(runtime)
            trials += 1
    plt.xlabel("Heuristic Type")
    plt.ylabel("Average A* Runtime (s)")
    plt.title("Average Runtime by Heuristic Type, 100 trials, dim = 101, p = .3")
    plt.bar(["Euclidean", "Manhattan", "Chebyshev"], [sum(euclidean_vals) / len(euclidean_vals), sum(manhattan_vals) / len(manhattan_vals), sum(chebyshev_vals) / len(chebyshev_vals)], color="#FF0000")           

#Heuristic running on the same maze for Q5
#execute 100 trails, each trial tests which heuristic runs faster(win) on the same maze
def plot_Q5():
    results = {EUCLIDEAN_DIST:0, MANHATTAN_DIST:0, CHEBYSHEV_DIST:0}
    run_time = {}
    trials = 0
    while trials <100:
        grid = generate_gridworld(101, 101, 0.3)
        for heuristic in (EUCLIDEAN_DIST, MANHATTAN_DIST, CHEBYSHEV_DIST):
            t1 = time.time()
            res = A_star(grid, (0, 0), (100, 100), heuristic)
            t2 = time.time()
            if not res:
                break
            run_time[heuristic] = t2-t1
        if not res:
            continue
        print(run_time)
        #return the key with min value in the dic, here the heuristic with min time
        winner = pd.Series(run_time).idxmin()
        results[winner] += 1
        trials += 1
    plt.xlabel("Heuristic Type")
    plt.ylabel("Total wins")
    plt.title("Performance analysis, 100 trials, dim = 101, p = .3")
    plt.bar(["Euclidean", "Manhattan", "Chebyshev"], results.values() , color="#FF0000")
    return results
    
#results = plot_Q5()        

def solvability_plot_4():
    probs = list(range(0, 101, 2))
    probs = list(map(lambda x : x / 100, probs))
    print(probs)
    solvability = [0] * len(probs)
	
    for i, prob in enumerate(probs):
        for trial in range(100):
            res = algorithmA(generate_gridworld(101, 101, prob), (0, 0), (100, 100), True, True)
            if res:
                solvability[i] += 1
    
    plt.xlabel("Probability")
    plt.ylabel("Solvability")
    plt.title("Solvability vs. Probability")
    plt.xticks([0, .25, .5, .75, 1])
    plt.plot(probs, solvability)
    return solvability

    
#Q6
#Best heuristic in Q5:Manhattan
#Density p=range(0,0.3)
"""Trajectory Length: Number of cells being traversed to reach the goal node(which might also include double counting cells when we backtrack) 
Length of Shortest Path in Final Discovered Gridworld: the path which we get from Running a single A* algorithm on the knowledge grid we got from running repeated A* algorithm
Length of Shortest Path in Full Gridworld:the path which we get from Running a single A* algorithm on the gridworld in which we know the status of blocked and unblocked cells.
Average Number of Cells Processed by Repeated A*: Number of cells that were popped from the fringe"""
def plot_Q6and7(has_four_way_vision):
    probs = list(range(0, 31, 2))
    probs = list(map(lambda x : x / 100, probs))
    trajectory_length = {prob: [] for prob in probs}
    length_discovered = {prob: [] for prob in probs}
    traj_over_length_disc = {prob: [] for prob in probs}
    disc_over_full = {prob: [] for prob in probs}
    
    length_full =[]
    Num_of_cell = {prob: [] for prob in probs}
    
    for i, prob in enumerate(probs):
        trial = 0
        while trial < 10:
            grid = generate_gridworld(101, 101, prob)
            res_unknown = algorithmA(grid, (0, 0), (100, 100), False, has_four_way_vision)
            if not res_unknown:
                continue
            #trajectory_length[prob].append(len(res_unknown[0]))
            Num_of_cell[prob].append(res_unknown[1])
            #shortest_discovered = A_star(res_unknown[2], (0, 0), (100, 100))
            #length_discovered[prob].append(len(shortest_discovered[0]))
            #traj_over_length_disc[prob].append(len(res_unknown[0]) / len(shortest_discovered[0]))
            #shortest_full = A_star(grid, (0, 0), (100, 100))
            #disc_over_full[prob].append(len(shortest_discovered[0]) / len(shortest_full[0]))
            #length_discovered = []
           # res_known =algorithmA(grid, (0, 0), (100, 100), True, has_four_way_vision)
            #length_full.append(len(res_known[0]))
            trial += 1
            
    # Plot 1
    #plt.title("Density vs. Average Trajectory Length (one way vision)")
    #plt.xlabel("Density")
    #plt.ylabel("Average Trajectory Length")
    #plt.plot(probs, list(map(lambda x: (sum(x) / len(x)), list(trajectory_length.values()))))
        
    # Plot 2
    #plt.title("Density vs. Average (Length of Trajectory / Length of Shortest Path in Final Discovered Gridworld (one way vision)")
    #plt.xlabel("Density")
    #plt.ylabel("Average (Length of Trajectory / Length of Shortest Path in Final Discovered Gridworld")
    #plt.plot(probs, list(map(lambda x: (sum(x) / len(x)), list(traj_over_length_disc.values()))))
            
    # Plot 3
    #plt.title("Density vs Average (Length of Shortest Path in Final Discovered Gridworld / Length of ShortestPath in Full Gridworld (one way vision)")
    #plt.xlabel("Density")
    #plt.ylabel("Average (Length of Shortest Path in Final Discovered Gridworld / Length of ShortestPath in Full Gridworld")
    #plt.plot(probs, list(map(lambda x: (sum(x) / len(x)), list(disc_over_full.values()))))
    
    # Plot 4
    plt.title("Density vs Average Number of Cells Processed by Repeated A* (one way vision)")
    plt.xlabel("Density")
    plt.ylabel("Average Number of Cells Processed by Repeated A*")
    plt.plot(probs, list(map(lambda x: (sum(x) / len(x)), list(Num_of_cell.values()))))



#print("Plot 4 solvability array:", solvability_plot_4())
#plot_data_5()    
#plot_Q6and7(True)
plot_Q6and7(False)

# Test algorithm
# is_grid_known is True for Questions 4, 5. It is False for later questions
# has_four_way_vision is True for all questions except 7
#shortest_path = algorithmA(generate_gridworld(5, 5, 0.3), (0,0), (4,4), False, True)
#test1
#grid = np.array([[0, 0, 0, 1],
#       [0, 0, 1, 0],
#       [0, 1, 0, 0],
#       [0, 0, 0, 0]])
#start = (0,0)
#end = (len(grid)-1,len(grid)-1)
#path_finder = A_star(grid, start, end)
#test2
#grid = generate_gridworld(10, 10, 0.2)
#start = (0,0)
#end = (len(grid)-1,len(grid)-1)
#path_finder = A_star(grid, start, end,1)
#A_path, A_knowledge = algorithmA(grid,start,end,is_grid_known=False, has_four_way_vision=True)
