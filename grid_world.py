#!/usr/bin/env python3

import numpy as np
from queue import PriorityQueue

# creating gridworld
def generate_gridworld(length,width,p):
    grid = np.random.choice([0,1],length*width, [1-p,p]).reshape(length,width)
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

def algorithmA(grid,start,end):
    pq=PriorityQueue()
    tieBreaker=0 # it is use to break tie of cells which have same f value.
    pq.put((0,tieBreaker,start)) # add start cell to the priority Queue.
    cameFrom={} # keep track of parent of cell
    g={eachcolumn:float("inf") for eachRow in grid for eachcolumn in eachRow}
    g[start]=0
    f={eachcolumn:float("inf") for eachRow in grid for eachcolumn in eachRow}
    start = (0,0)
    #goal = (length,width)
    f[start]=hureisticValue(start,goal)

