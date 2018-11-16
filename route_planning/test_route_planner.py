
# Import our search algorithms, simulation_resources modules
from search_algorithms import *
from shared.simulation_resources import *


#######################################
# TEST ROUTE PLANNING ALGORITHMS HERE #
#######################################

# Generate a random GridWorld
my_grid = GridWorld(map_type="RMC")
my_grid.show_image()

# Test each algorithm
my_grid.test_search_algo(A_star)



