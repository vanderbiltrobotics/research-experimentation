# Include required packages
from matplotlib import pyplot as plt
import numpy as np
from math import *
from random import *
from datetime import datetime

# ----------------------------------------------------------------------------
# GridWorld class
# ----------------------------------------------------------------------------
# Represents a world using a 2D grid. Every position in the world is either
# empty (0) or contains an obstacle (1). Contains methods to display the world
# as a black and white image or to display a path overlayed on the grid
# ----------------------------------------------------------------------------


class GridWorld:

    # Randomly generates a new GridWorld with specified parameters
    def __init__(self, length=576, width=369, num_obs=4, min_obs_size=10, max_obs_size=30, map_type="default"):

        # First generate empty map
        new_grid = np.zeros((length, width), np.uint8)

        # Obstacles can only be added between these bounds
        upper_bound = 0
        lower_bound = length

        # If map type is RMC, override any other variables that are set
        if map_type == "RMC":
            upper_bound = length / 3
            lower_bound = 2 * upper_bound
            length = 576
            width = 369
            num_obs = 4
            min_obs_size = 10
            max_obs_size = 30

        # Add obstacles randomly throughout grid
        for i in range(num_obs):

            # Randomly generate center coords of object
            center_x = randint(max_obs_size / 2, width - max_obs_size / 2)
            center_y = randint(upper_bound + (max_obs_size / 2), lower_bound - (max_obs_size / 2))

            # Randomly generate size of the object
            mu = (min_obs_size + max_obs_size) / 2.0
            sigma = (max_obs_size - min_obs_size) / 6.0
            obs_size = gauss(mu, sigma)

            # Limit object size
            obs_size = min_obs_size if obs_size < min_obs_size else obs_size
            obs_size = max_obs_size if obs_size > max_obs_size else obs_size
            obs_rad = obs_size / 2.0

            # Add obstacle to map
            for x in range(0, width):
                for y in range(length):
                    if sqrt(pow((x - center_x), 2) + pow((y - center_y), 2)) < obs_rad:
                        new_grid[y, x] = 1

        # If RMC map, add collector bin as obstacle

        # Initialize class variables
        self.grid = new_grid
        self.length = length
        self.width = width
        self.num_obs = num_obs
        self.min_obs_size = min_obs_size
        self.max_obs_size = max_obs_size
        self.lower_bound = lower_bound
        self.upper_bound = upper_bound
        self.map_type = map_type

    def __repr__(self):
        print "<" + str(self.length) + "x" + str(self.width) + " GridWorld object>"

    def __str__(self):
        return self.__repr__()

    def show_image(self):
        plt.imshow(np.ones(self.grid.shape, np.uint8) - self.grid, interpolation='nearest', cmap='gray')
        plt.show()

    def show_path(self, start_pos, end_pos, path):

        # Display world grid
        plt.imshow(np.ones(self.grid.shape, np.uint8) - self.grid, interpolation='nearest', cmap='gray')

        # Plot start and end points
        plt.plot(start_pos[0], start_pos[1], 'bo')
        plt.plot(end_pos[0], end_pos[1], 'go')

        # Plot path over grid
        for i in range(len(path) - 1):
            plt.plot([path[i][0], path[i + 1][0]], [path[i][1], path[i + 1][1]], linewidth=2.0, color='r')

        # Show
        plt.show()

    # Generate a starting position within Grid - if RMC mode, limit to starting region
    def get_rand_start(self):
        start_x = randint(0, self.width-1)
        start_y = randint(0, self.length-1) if self.map_type != "RMC" else randint(self.lower_bound, self.length-1)

        # Make sure we aren't starting inside an obstacle
        while self.grid[start_y, start_x] == 1:
            start_x = randint(0, self.width - 1)
            start_y = randint(0, self.length - 1) if self.map_type != "RMC" else randint(self.lower_bound,
                                                                                         self.length - 1)
        return [start_x, start_y]

    # Generate an ending position within Grid
    def get_rand_end(self):
        end_x = randint(0, self.width - 1)
        end_y = randint(0, self.length - 1)

        # Make sure we aren't starting inside an obstacle
        while self.grid[end_y, end_x] == 1:
            end_x = randint(0, self.width - 1)
            end_y = randint(0, self.length - 1)

        return [end_x, end_y]

    # Tests a provided search algorithm with the provided points and the GridWorld objects grid
    def test_search_algo(self, search_algo, display_results=True):

        # Create random start and end points
        start_pos = self.get_rand_start()
        end_pos = self.get_rand_end()

        # Record how long algorithm takes to run
        start_time = datetime.now()

        # Run provided search algorithm on self, get list of points
        path_points = search_algo(self.grid, start_pos, end_pos)

        # Measure elapsed time
        elapsed = datetime.now() - start_time
        print elapsed

        # Display resulting path
        if display_results:
            self.show_path(start_pos, end_pos, path_points)


# ----------------------------------------------------------------------------
# Robot class
# ----------------------------------------------------------------------------
# Represents a world using a 2D grid. Every position in the world is either
# empty (0) or contains an obstacle (1). Contains methods to display the world
# as a black and white image or to display a path overlayed on the grid
# ----------------------------------------------------------------------------

class Robot:

    def __init__(self):
