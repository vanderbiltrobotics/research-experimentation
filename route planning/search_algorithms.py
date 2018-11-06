
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
# 'start_point' and 'target_point' are x, y pairs
# 'path' is a list of [x, y] pairs, all turning points along route
#
# ALgorithms can be tested using the 'test_search_algo' method of the
# GridWorld class in test_route_planner.py
#
# ------------------------------------------------

# Import required packages
import numpy as np

# A star search algorithm for path planning
def A_star(grid, start_pos, end_pos):
    path = []
    return path

# Theta star search algorithm for path planning - any angle extension of A star
def theta_star(grid, start_pos, end_pos):
    path = []
    return path