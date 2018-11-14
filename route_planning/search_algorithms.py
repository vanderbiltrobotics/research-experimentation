
# ------------------------------------------------
# DEFINE SEARCH ALGORITHMS FOR ROUTE PLANNING HERE
# ------------------------------------------------
#
# Every algorithm should have the following form:
#
# def search_algo(grid, start_point, target_point [, additional_params]):
#   [function body]
#   return path
#
# 'grid' is a 2-D numpy array where 1s represent obstacles
# 'start_point' and 'target_point' are [x, y] pairs
# 'path' is a list of [x, y] pairs, all turning points along route
#
# ALgorithms can be tested using the 'test_search_algo' method of the
# GridWorld class in test_route_planner.py
#
# ------------------------------------------------

# Import required packages
import numpy as np
import inspect
from collections import deque
from operator import itemgetter
from math import *

#####################
# CLASSES           #
#####################

class Point:

    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.x = position[0]
        self.y = position[1]
        self.g = 0
        self.h = 0
        self.f = self.g+self.h

    def cost(self):
        self.f = self.g+self.h
        return self.f



#####################
# HELPER FUNCTIONS #
#####################

# Convenience function, returns a list of the neighbors of a point
# in a grid with the specified dimensions
def get_neighbors(dims, point):
    dims_xy = [dims[1], dims[0]]  # Order dimensions this way to avoid confusion
    neighbors = []
    if isinstance(point, Point):
        if point.x > 0:
            point.x = point.x - 1
            neighbors.append(point)
        if point.y < dims_xy[0] - 1:
            point.x = point.x + 1
            neighbors.append(point)
        if point.y > 0:
            point.y = point.y - 1
            neighbors.append(point)
        if point.y < dims_xy[1] - 1:
            point.y = point.y + 1
            neighbors.append(point)
    else:
        if point[0] > 0:
            neighbors.append([point[0] - 1, point[1]])
        if point[0] < dims_xy[0] - 1:
            neighbors.append([point[0] + 1, point[1]])
        if point[1] > 0:
            neighbors.append([point[0], point[1] - 1])
        if point[1] < dims_xy[1] - 1:
            neighbors.append([point[0], point[1] + 1])
    return neighbors

# Returns distance between two points - useful heuristic for informed search
def get_dist(p1, p2):
    return sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[0])**2)


#####################
# SEARCH ALGORITHMS #
#####################

# Simple breadth / depth first search algorithm - very slow
# Uses depth first by default
def uninformed_search(grid, start_pos, end_pos, mode="DEPTH"):

    # Matrix to mark nodes as visited
    visited = np.array(grid, copy=True)

    # Initialize frontier as queue, add initial pos
    frontier = deque()
    frontier.appendleft([start_pos])

    # Search until a path is found or all nodes visited
    counter = 0
    while len(frontier) > 0:

        # Dequeue next node
        next = frontier.pop()   # 'next' is a full path
        last = next[-1]         # get last node visited on this path

        # Check for goalness
        if last == end_pos:
            print "Paths checked: " + str(counter)
            return next

        # Update visited
        visited[last[1], last[0]] = 1      # row, col (y, x)

        # Add un-visited neighbors to Queue
        for neighbor in get_neighbors(grid.shape, last):
            if visited[neighbor[1], neighbor[0]] == 0:        # row, col (y, x)
                new_path = list(next)
                new_path.append(neighbor)
                if mode == "DEPTH":
                    frontier.appendleft(new_path)
                else:
                    frontier.append(new_path)

        counter += 1

    # No path found
    print "--No path found--"
    return []


# Greedy best first search - only uses distance heuristic
def greedy_bfs(grid, start_pos, end_pos):

    # Matrix to mark nodes as visited
    visited = np.array(grid, copy=True)

    # Initialize frontier as queue, add initial pos
    frontier = []
    frontier.append([[start_pos], get_dist(start_pos, end_pos)])

    # Search until a path is found or all nodes visited
    counter = 0
    while len(frontier) > 0:

        # Dequeue next node
        next = frontier.pop()   # 'next' is a full path
        last = next[0][-1]      # get last node visited on this path

        # Check for goalness
        if last == end_pos:
            print "Paths checked: " + str(counter)
            return next[0]                             # Just return the points, not the associated distances

        # Update visited
        visited[last[1], last[0]] = 1      # row, col (y, x)

        # Add un-visited neighbors to Queue
        for neighbor in get_neighbors(grid.shape, last):
            if visited[neighbor[1], neighbor[0]] == 0:        # row, col (y, x)
                new_path = list(next[0])
                new_path.append(neighbor)
                new_path = [new_path, get_dist(neighbor, end_pos)]
                frontier.append(new_path)

        # Now sort the paths based on distance heuristic
        frontier.sort(key=itemgetter(1), reverse=True)

        # Increment counter
        counter += 1

    # No path found
    print "--No path found--"
    return []


# A star search algorithm for path planning
def A_star(grid, start_pos, end_pos):
    path = []
    start = Point(None, start_pos)
    end = Point(None, end_pos)
    unchecked = [start]
    current = start
    checked = []
    while not len(unchecked)==0:
        #TODO: add sort
        current = unchecked[0]
        if current.x == end.x and current.y == end.y:
            break
        unchecked.remove(current)
        checked.append(current)
        for neighbour in get_neighbors(grid,current):
            if neighbour in checked:
                #if checked
                continue
            tempG = current.g+1
            #1 as the cost of moving one grid
            else if (neighbour not in unchecked) or (tempG < neighbour.g):
                #it has not entered the unchecked yet, or we just found a path of lower cost
                neighbour.g = tempG
                #update the current cost
                neighbour.h = get_dist([neighbour.x,neighbour.y],[end.x,end.y])
                neighbour.cost()
                unchecked.append(neighbour)
        #checked all unchecked list
    #should have searched the whole map and reached the end point here.
    #now trace back to the starting position and out put the points in the path
    while not current.parent == None:
        path.append(current)
        current=current.parent
    return path

# Theta star search algorithm for path planning - any angle extension of A star
def theta_star(grid, start_pos, end_pos):
    path = []
    return path