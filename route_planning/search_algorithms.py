
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
import matplotlib.pyplot as plt
import inspect
import copy
from collections import deque
from operator import itemgetter
from math import *

#####################
# HELPER FUNCTIONS #
#####################

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
    return sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)

def construct(cameFrom, path, c):
    if not(cameFrom[(c[0],c[1])] is None):
        path.append(c)
        c = cameFrom[(c[0],c[1])]
        construct(cameFrom,path,c)


def lineofsight(parent, neighbor, grid):
    yp = parent[1]
    xp = parent[0]
    yn = neighbor[1]
    xn = neighbor[0]
    dy = yn - yp
    dx = xn - xp
    sy = -1 if dy < 0 else 1
    sx = -1 if dx < 0 else 1
    dy *= sy
    dx *= sx
    f = 0
    if dx >= dy:
        while xp != xn:
            f += dy
            if f>= dx:
                if grid[yp+((sy-1)/2),xp+((sx-1)/2)]:
                    return False
                yp += sy
                f -= dx
            if not f and grid[yp+((sy-1)/2),xp+((sx-1)/2)]:
                return False
            if not dy and grid[yp,xp+((sx-1)/2)] and grid[yp-1,xp+((sx-1)/2)]:
                return False
            xp += sx
    else:
        while yp != yn:
            f += dx
            if f>=dy:
                if grid[yp+((sy-1)/2),xp+((sx-1)/2)]:
                    return  False
                xp += sx
                f -= dy
            if not f and grid[yp+((sy-1)/2),xp+((sx-1)/2)]:
                return False
            if not dx and grid[yp+((sy-1)/2),xp] and grid[yp+((sy-1)/2),xp-1]:
                return False
            yp+=sy
    return True

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
def A_star(grid, start, end):
    openList = [] #in form of (point_coordinates, f_score)
    openList.append((start, get_dist(start, end)))
    closedList = []
    cameFrom = {}#dictionary in form of point_coordinates:parent coordinate
    cameFrom[(start[0], start[1])] = None
    gScore = {}#dictionary key:coordinate,item:gscore
    gScore[(start[0], start[1])] = 0
    #while have item on the plan
    while not (len(openList) is 0):
        #get the most efficient next step to go, sort by f
        openList = sorted(openList,key=lambda x: x[1])
        curr = openList.pop(0)
        closedList.append(curr)
        #get the coordinate
        current = curr[0]
        if (current[0] == end[0]) and (current[1] == end[1]):
            path = []
            construct(cameFrom,path,current)
            return path
        neighbors = get_neighbors([576, 369], current)
        for neighbor in neighbors:
            if grid[neighbor[1], neighbor[0]] != 1:
                #gscore for the neighbor we are considering, parent_gscore+1
                tempG = gScore[(current[0], current[1])] + get_dist(current, neighbor)
                #if the neighbor point has already been visited, distance is always the same, only comparing steps taken(g)
                #if this path is not as effecient, continue, skip this
                if (neighbor in [item[0] for item in closedList]) and (tempG >= gScore[(neighbor[0], neighbor[1])]):
                    continue
                #if this is a new point, or this is a more efficient path
                if (not (neighbor in [item[0] for item in closedList])) or (tempG < gScore[(neighbor[0], neighbor[1])]):
                    #update path to get neighbor
                    cameFrom[(neighbor[0], neighbor[1])] = current
                    gScore[(neighbor[0], neighbor[1])] = tempG
                    #append neighbot to the plan
                    openList.append((neighbor, gScore[(neighbor[0], neighbor[1])] + 1000*get_dist(neighbor, end)))
    return 0

# Theta star search algorithm for path planning - any angle extension of A star
def theta_star(grid, start_pos, end_pos):
    openList = []
    openList.append((start_pos, get_dist(start_pos, end_pos)))
    closedList = []
    cameFrom = {}
    cameFrom[(start_pos[0],start_pos[1])] = None
    gScore = {}
    gScore[(start_pos[0],start_pos[1])] = 0
    while not(len(openList) == 0):
        #TODO hScore integration
        openList = sorted(openList, key=lambda x: x[1])
        curr = openList.pop(0)
        closedList.append(curr)
        curr = curr[0]
        if(curr[0]==end_pos[0]) and (curr[1]==end_pos[1]):
            path = []
            construct(cameFrom,path,curr)
            return path
        for neighbor in get_neighbors([576, 369], curr):
            if grid[neighbor[1],neighbor[0]] != 1:
                tempG = gScore[(curr[0], curr[1])] + get_dist(curr, neighbor)
                if (not(neighbor in [item[0] for item in closedList]) or (tempG < gScore[((neighbor[0], neighbor[1]))])):
                    cameFrom[(neighbor[0], neighbor[1])] = curr
                    gScore[(neighbor[0], neighbor[1])] = tempG
                    openList.append((neighbor, gScore[(neighbor[0], neighbor[1])] + 100*get_dist(neighbor, end_pos)))
                    parent = cameFrom[(curr[0], curr[1])] if not (curr[0] == start_pos[0] and curr[1] == start_pos[1]) else curr
                    tempG = gScore[(parent[0], parent[1])] + get_dist(parent, neighbor)
                    if lineofsight(parent, neighbor, grid):
                        if tempG < gScore[(neighbor[0], neighbor[1])]:
                            cameFrom[(neighbor[0], neighbor[1])] = parent
                            gScore[(neighbor[0], neighbor[1])] = tempG
                            openList.append((neighbor, tempG + 100*get_dist(neighbor, end_pos)))
    return None
