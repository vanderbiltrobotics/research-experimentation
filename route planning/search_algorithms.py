
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

# Convenience function, returns a list of the neighbors of a point
# in a grid with the specified dimensions
def get_neighbors(dims, point):
    dims_xy = [dims[1], dims[0]]  # Order dimensions this way to avoid confusion
    neighbors = []
    if point[0] > 0:
        neighbors.append([point[0] - 1, point[1]])
    if point[0] < dims_xy[0] - 2:
        neighbors.append([point[0] + 1, point[1]])
    if point[1] > 0:
        neighbors.append([point[0], point[1] - 1])
    if point[1] < dims_xy[1] - 2:
        neighbors.append([point[0], point[1] + 1])
    return neighbors

# Simple breadth first search algorithm - slow but finds the best solution
def breadth_first(grid, start_pos, end_pos):

    # Matrix to mark nodes as visited
    visited = np.array(grid, copy=True)

    # Initialize frontier as queue, add initial pos
    frontier = deque()
    frontier.appendleft([start_pos])

    # Search until a path is found or all nodes visited
    while len(frontier) > 0:

        # Dequeue next node
        next = frontier.pop()   # 'next' is a full path
        last = next[-1]         # get last node visited on this path

        # Check for goalness
        if last == end_pos:
            return next

        # Update visited
        visited[last[1], last[0]] = 1      # row, col (y, x)

        # Add un-visited neighbors to Queue
        for neighbor in get_neighbors(grid.shape, last):
            if visited[neighbor[1], neighbor[0]] == 0:        # row, col (y, x)
                new_path = list(next)
                new_path.append(neighbor)
                frontier.appendleft(new_path)

    # No path found
    return []


# A star search algorithm for path planning
def A_star(grid, start_pos, end_pos):
    path = []
    return path

# Theta star search algorithm for path planning - any angle extension of A star
def theta_star(grid, start_pos, end_pos):
    path = []
    return path