#!/usr/bin/env python3

import numpy as np
from queue import PriorityQueue
import sys

# creating gridworld
def generate_gridworld(rows,cols,probability):
    grid = np.random.choice([False,True],rows*cols, p=[1.0-probability,probability]).reshape(rows,cols)
    grid[0][0] = False
    grid[-1][-1] = False
    print(grid)
    cell_list = []
    for i in range(rows):
        cell_list.append([])
        for j in range(cols):
            cellOBJ = Cell(i,j,rows)
            cellOBJ.blocked = grid[i][j]
            cell_list[i].append(cellOBJ)
    return cell_list

#Cell class
class Cell:
    def __init__(self, row, col, totalRows):
        self.row=row
        self.col=col
        self.totalRows=totalRows
        self.neighbors=[]
        self.visited=False
        self.blocked=False

    def getPos(self):
        return self.col, self.row
    
    """ (0,0)
                row-1
        col-1   cell    col+1
                row+1
    (y,x)
    """
       
    def neighborsOf(self, grid):
        self.nbrs=[]    

        #   upper nbr - row-1
        # if cell has row not 0 then it does not have upper cell
        # if upper cell is not barrier then append the upper cell as neighbour.
        if self.row>0 and not grid[self.row-1][self.col].blocked:
            self.nbrs.append(grid[self.row-1][self.col])

        #   bottom nbr - row+1
        # if cell is not the last row then it has buttom row
        # if bottom cell is not barrier then append the bottom cell as neighbour.
        if self.row< self.totalRows-1 and not grid[self.row+1][self.col].blocked:
            self.nbrs.append(grid[self.row+1][self.col])
        #   left nbr - col-1
        # if cell is not the first col then it has left cell
        # if left cell is not barrier then apend the left cell as neighbour.
        if self.col>0 and not grid[self.row][self.col-1].blocked:
            self.nbrs.append(grid[self.row][self.col-1])
        #   right nbr - col+1
        # if cell is not the last col then it has right cell
        # if right cell is not barrier then apend the right cell as neighbour.
        if self.col < self.totalRows-1 and not grid[self.row][self.col+1].blocked:
            self.nbrs.append(grid[self.row][self.col+1])

    def __lt__(self, other):
        return False

def FindShortesPath(parent,currentCell):
    shortest_path = [] #to store position of shortest path
    while currentCell in parent: # here we backtrack from end to start
        currentCell=parent[currentCell]
        shortest_path.append(currentCell.getPos())
    return shortest_path

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
    g={eachcolumn:float("inf") for eachRow in grid for eachcolumn in eachRow}
    g[start]=0
    f={eachcolumn:float("inf") for eachRow in grid for eachcolumn in eachRow}
    f[start]=hureisticValue(start.getPos(),end.getPos())
    #pq=set([ (f[start], start) ]) # add start cell and distance information to the priority queue.
    pq = PriorityQueue()
    pq.put((0,start))
    parent={} # keep track of parent of cell
    visited={start} # it is a set which provide the uniqueness, means it is ensure that not a single cell visit more than onece.
    while not pq.empty(): # now until Priority Queue is not empty loop 
        currentCell= pq.get()[1] # get first item of priority Queue
        visited.remove(currentCell) # also remove that item from the set
        if currentCell==end: # if item is goal then
            shortest_path = FindShortesPath(parent,currentCell)
            shortest_path.reverse()
            print(shortest_path) # print the shortest path
            return True
        for neighbours in currentCell.nbrs: # adding neighbours which are not visited, in other words which are not present in set
            # beacuse those are unvisited, we just treat the set as visited array.
            #and update its g anf f values of neighbours of cells.
            tempG=g[currentCell]+1 # as it is a grid not graph so distance between cells is 1
            if tempG<g[neighbours]: # here we update the g of neighbours, if previous g is greater then update g to latest calculated g which is tempG. 
                parent[neighbours]=currentCell # keep track of parent of each neighbours
                g[neighbours]=tempG
                f[neighbours]=tempG+hureisticValue(neighbours.getPos(), end.getPos()) # calculate and update f of each neighbours.
                if neighbours not in visited: # if neighbours are not visited so they are unique
                    pq.put((f[neighbours],neighbours)) #add neighbours to Priority Queue
                    visited.add(neighbours) #add neighbours to set
    return False

def main(argv):
    grid = generate_gridworld(20,20,.1)
    start = grid[0][0]
    end = grid[-1][-1]
    for eachrow in grid:
        for eachcolumn in eachrow:
            eachcolumn.neighborsOf(grid)
    path_finder = algorithmA(grid, start, end)
    
if __name__ == "__main__":
  main(sys.argv)
        