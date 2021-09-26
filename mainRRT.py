import numpy as np
import matplotlib.pyplot as plt
from path_planner import PathPlanner
from grid import CostMap
from math import inf
import random
import time

""" RRT:
1: sample a random point uniformly over the grid;
2: decide whether to connect or not: it should be connected to the closest node already
   in the path (linear search), but it shouldn't be farther than a given radius d from it
   (hence you may take the farthest point on that way within the radius. It doesn't change
   who is the closest neighbor).
   Also, if the path to the closest neighbor is blocked by an obstacle, discard the new
   point. (*discard it right away* or try to connect it to another neighbor?)
3: keep sampling until you reach a radius d of the goal point. (Don't stop the algorithm,
   just signal that you found a feasible path). 

    RRT*:
1: sample just as in RRT
2: decide on the connection of the new point the same way as in RRT, but with an extra 
   last step. If you decide to connect it to some node in the path, search over a radius R
   for a more suitable connection (find the argmin cost(x) + cost(x, xnew) over all x in 
   that radius such that the edge(x, xnew) is possible). 
3: the criterion for rebranching the tree is that the cost of a node via a path passing 
   through the new node is smaller, then we rebranch that node to make the new node its
   parent. 

"""

def plot_path(cost_map, start, goal, path, filename, save_fig=True, show_fig=True, fig_format='png'):
    """
    Plots the path.

    :param cost_map: cost map.
    :param start: start position.
    :param goal: goal position.
    :param path: path obtained by the path planning algorithm.
    :param filename: filename used for saving the plot figure.
    :param save_fig: if the figure will be saved to the hard disk.
    :param show_fig: if the figure will be shown in the screen.
    :param fig_format: the format used to save the figure.
    """
    plt.matshow(cost_map.grid)
    x = []
    y = []
    for point in path:
        x.append(point[1])
        y.append(point[0])
    plt.plot(x, y, linewidth=1)
    
    plt.plot(start[1], start[0], 'y*', markersize=8)
    plt.plot(goal[1], goal[0], 'rx', markersize=8)

    plt.xlabel('x / j')
    plt.ylabel('y / i')
    
    plt.title('Basic RRT')
    
    if show_fig:
        plt.show()


# Environment's parameters
WIDTH = 160
HEIGHT = 120
OBSTACLE_WIDTH = 20
OBSTACLE_HEIGHT = 15
NUM_OBSTACLES = 20

cost_map = CostMap(WIDTH, HEIGHT)
# Initializing the random seed so we have reproducible results
# Please, do not change the seed
random.seed(15)
# Create a random map
cost_map.create_random_map(OBSTACLE_WIDTH, OBSTACLE_HEIGHT, NUM_OBSTACLES)
# Create the path planner using the cost map
path_planner = PathPlanner(cost_map)

##########################
num_nodes = int(input("How many samples? "))

while num_nodes > 0:
    # sample num_nodes nodes
    # plot the map with the path
    #


##########################

for i in range(num_iterations):
    problem_valid = False
    while not problem_valid:
        # Trying to generate a new problem
        start_position = (random.randint(0, HEIGHT - 1), random.randint(0, WIDTH - 1))
        goal_position = (random.randint(0, HEIGHT - 1), random.randint(0, WIDTH - 1))
        # If the start or goal positions happen to be within an obstacle, we discard them and
        # try new samples
        if cost_map.is_occupied(start_position[0], start_position[1]):
            continue
        if cost_map.is_occupied(goal_position[0], goal_position[1]):
            continue
        if start_position == goal_position:
            continue
        problem_valid = True
    tic = time.time()
    if algorithm == 'dijkstra':
        path, cost = path_planner.dijkstra(start_position, goal_position)
    elif algorithm == 'greedy':
        path, cost = path_planner.greedy(start_position, goal_position)
    else:
        path, cost = path_planner.a_star(start_position, goal_position)
    # if path is not None and len(path) > 0:
    path_found = True
    toc = time.time()
    times[i] = toc - tic
    costs[i] = cost
    plot_path(cost_map, start_position, goal_position, path, '%s_%d' % (algorithm, i), save_fig, show_fig, fig_format)


# Print Monte Carlo statistics
print(r'Compute time: mean: {0}, std: {1}'.format(np.mean(times), np.std(times)))
if not (inf in costs):
    print(r'Cost: mean: {0}, std: {1}'.format(np.mean(costs), np.std(costs)))
