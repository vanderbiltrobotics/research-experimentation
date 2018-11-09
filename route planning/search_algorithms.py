
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
from collections import deque
from operator import itemgetter
from math import *


# Convenience function, returns a list of the neighbors of a point
# in a grid with the specified dimensions
def get_neighbors(dims, point):
    dims_xy = [dims[1], dims[0]]  # Order dimensions this way to avoid confusion
    neighbors = []
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
    return sqrt((p2[0] - p1[0])**2 + (p2[1] - p2[0])**2)


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
    return path

# Theta star search algorithm for path planning - any angle extension of A star
def theta_star(grid, start_pos, end_pos):
    path = []
    return path