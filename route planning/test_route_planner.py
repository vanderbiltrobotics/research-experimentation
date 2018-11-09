
# Include required packages
from matplotlib import pyplot as plt
import numpy as np
from math import *
import random
from datetime import datetime

# Import our search algorithms module
from search_algorithms import *


class GridWorld:

    # Randomly generates a new GridWorld with specified parameters
    def __init__(self, length, width, num_obs, min_obs_size, max_obs_size):

        # First generate empty map
        map = np.zeros((length, width), np.uint8)

        # Add obstacles to middle third of grid
        upper_bound = length / 3
        lower_bound = 2 * upper_bound

        for i in range(num_obs):

            # Randomly generate center coords of object
            center_x = random.randint(max_obs_size / 2, width - max_obs_size / 2)
            center_y = random.randint(upper_bound + (max_obs_size / 2), lower_bound - (max_obs_size / 2))

            # Randomly generate size of the object
            mu = (min_obs_size + max_obs_size) / 2.0
            sigma = (max_obs_size - min_obs_size) / 6.0
            obs_size = random.gauss(mu, sigma)

            # Limit object size
            obs_size = min_obs_size if obs_size < min_obs_size else obs_size
            obs_size = max_obs_size if obs_size > max_obs_size else obs_size
            obs_rad = obs_size / 2.0

            # Add obstacle to map
            for x in range(0, width):
                for y in range(upper_bound, lower_bound):
                    if sqrt(pow((x - center_x), 2) + pow((y - center_y), 2)) < obs_rad:
                        map[y, x] = 1

        # Assign starting location
        start_x = random.randint(0, width)
        start_y = random.randint(lower_bound, length)

        # Assign ending location
        target_x = random.randint(0, width)
        target_y = random.randint(0, upper_bound)

        # Initialize class variables
        self.grid = map
        self.robot_pos = [start_x, start_y]
        self.target_pos = [target_x, target_y]

    def __repr__(self):
        print self.grid

    def __str__(self):
        return str(self.grid)

    def show_image(self):
        plt.imshow(np.ones(self.grid.shape, np.uint8) - self.grid, interpolation='nearest', cmap='gray')
        plt.show()

    def show_path(self, path):

        # Display world grid
        plt.imshow(np.ones(self.grid.shape, np.uint8) - self.grid, interpolation='nearest', cmap='gray')

        # Plot start and end points
        plt.plot(self.robot_pos[0], self.robot_pos[1], 'bo')
        plt.plot(self.target_pos[0], self.target_pos[1], 'go')

        # Plot path over grid
        for i in range(len(path) - 1):
            plt.plot([path[i][0], path[i+1][0]], [path[i][1], path[i+1][1]], linewidth=2.0, color='r')

        # Show
        plt.show()

    def test_search_algo(self, search_algo, display_results=True):

        # Record how long algorithm takes to run
        start_time = datetime.now()

        # Run provided search algorithm on self, get list of points
        path_points = search_algo(self.grid, self.robot_pos, self.target_pos)

        # Measure elapsed time
        elapsed = datetime.now() - start_time
        print elapsed

        # Display resulting path
        if display_results:
            self.show_path(path_points)



#######################################
# TEST ROUTE PLANNING ALGORITHMS HERE #
#######################################

# Generate a random GridWorld
my_grid = GridWorld(30, 15, 2, 2, 5)
my_grid.show_image()

# Test each algorithm
my_grid.test_search_algo(breadth_first)



