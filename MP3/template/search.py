"""
This file contains search functions.
"""
# Search should return the path and the number of states explored.
# The path should be a list of tuples in the form (alpha, beta, gamma) that correspond
# to the positions of the path taken by your search algorithm.
# Number of states explored should be a number.
# maze is a Maze object based on the maze from the file specified by input filename
# searchMethod is the search method specified by --method flag (bfs,astar)
# You may need to slight change your previous search functions in MP1 since this is 3-d maze

from collections import deque
import heapq

# Search should return the path and the number of states explored.
# The path should be a list of MazeState objects that correspond
# to the positions of the path taken by your search algorithm.
# Number of states explored should be a number.
# maze is a Maze object based on the maze from the file specified by input filename
# searchMethod is the search method specified by --method flag (astar)
# You may need to slight change your previous search functions in MP2 since this is 3-d maze



# You will be moving an alien robot based on this idea. Specifically, the robot lives in 2D and has three degrees of freedom:

# It can move the (x,y) position of its center of mass.
# It can switch between three forms: a long horizontal form, a round form, and a long vertical form.
# Notice that the robot cannot rotate. Also, to change from the long vertical form to/from the long horizontal form, the robot must go through the round form, i.e., it cannot go from its vertical oblong shape to its horizontal oblong shape directly.

   

# The round form is a disk (or a filled circle). The horizontal or vertical long form is a "sausage" shape, defined by a line segment and a distance "d", i.e., any point within a given distance "d" of the LINE SEGMENT between the alien's head and tail is considered to be within the alien .

# For each planning problem you will be given a 2D environment specified by:

# The size of the workspace.
# The widths of the long and round forms.
# The starting (x,y) center-of-mass position and shape of the alien.
# A list of goals, each of which is defined by disk described by an (x,y) position and a radius.
# A set of obstacles, each of which is a line segment.
# You need to find a shortest path for the alien robot from its starting position to ONE of the goal positions.

# The tricky bit is that the alien may not pass through any of the obstacles. Also, no part of the robot should go outside the workspace. So configurations like the following are NOT allowed:

 

# Also note that, since we are discretizing the state space, we want to add a safety buffer equal to the half diagonal of a square whose side length is the granularity we're using to digitize our map. Specifically, if the granularity is G, we want our alien to be at least G2√ units way from any obstacles and the same distance away from the edges of the workspace. This ensures that the alien will not accidentally bump into an obstacle of go out of bounds while executing its path in the real world. This is important for, configurations like the one below, where the granularity is set to 10 pixels:



# Note that it seems that the alien turned red despite not visibly colliding with any wall. That is because at this coarse granularity, the safety buffer is very large. Keep that in mind as you visually debug later (this effect should be less perceptible at finer granularities).

# We consider the maze solved once any part of the alien's body has contacted the goal position (i.e. if any part of any of the objectives is within the alien area), as long as the alien is not violating any constraint (i.e., not out of bounds or touching a wall). Here are some valid "solved" states:
 

# Note that for reaching the goal, we DO NOT apply the collision buffer we use for obstacles and out-of-bounds checking, as we want to guarantee that we actually touch the goal.

# To solve this MP3, you will go through three main steps:
# Adapt your MP2 State and A* code to be able to search in three dimensions.
# This is primarily a housekeeping activity to ensure your code is not hardcoded to work for a specific type of search, but can handle generic search spaces.
# Compute a configuration space (Maze) from the original alien motion space. This configuration space is 3D because the alien robot has three degrees of freedom (x,y centroid position and shape).
# Convert that configuration space into a 3D version of MP2's map format and use your previously written 3D A* code to compute the shortest path in this configuration space map.
# In order to represent the alien's configuration space as a 3D maze, you will digitize the alien's current shape and 2D movement patterns. Each shape of the alien ('Vertical', 'Ball', 'Horizontal') can be represented by a "level" in the maze, while the walls in the maze are determined by the configurations where the current shape collides with obstacles. It is important to remember that the robot's configuration is comprised of both its (x,y) centroid position and its shape! In the mazes we want you to solve, the robot will have to switch its shape to reach the goal.

# Part 0: Understanding Map configuration
# Each scene (or problem) is defined in a settings file. The settings file must specify the alien's inital position, geometry of the alien in its different configurations, the goals and their geometry and the edges of the workspace. Here is a sample scene configuration:
#   [Test1]
#   Window : (300, 200)                     # (Width, Height)
#   Obstacles : [
#                   (0,100,100,100),  #(startx,starty,endx,endy)
#                   (0,140,100,140),
#                   (100,100,140,110),
#                   (100,140,140,130),
#                   (140,110,175,70),
#                   (140,130,200,130),
#                   (200,130,200,10),
#                   (200,10,140,10),
#                   (175,70,140,70),
#                   (140,70,130,55),
#                   (140,10,130,25),
#                   (130,55,90,55),
#                   (130,25,90,25),
#                   (90,55,90,25)
#               ]
#   Goals : [
#               (110, 40, 10)               # (x-coordinate, y-coordinate, radius)
#           ]
#   Lengths: [40,0,40]
#   Widths: [11,25,11]
#   StartPoint: [30,120]
# Window: The window size for the given example map is 300x200 pixels. Note that (0, 0) is the top-left corner and (300, 200) is the bottom-right corner .
# Lengths: The lengths of the robot are in the form ['Horizontal Length', 'Ball length (always 0)', 'Vertical Length']. The length of the robot is defined by the distance between its head and tail
# Widths: widths of the robot represent the radius of the circle that is added to the line segment "body" of the alien, i.e.. how far away from the line segment defining the body is still considered to be "inside" the robot. These are ordered in the same manner as the Lengths
# Obstacles: There are many walls in the maze which are represented by a list of endpoints for line segments in the format (startx, starty, endx, endy)
# A list of disk goals specified in the form (x,y,radius). In this particular maze, there is one goal at (110,40) that has radius 10 pixels.
# The alien is set to start at the position (30,120), in its default disk configuration
# The name of this map is Test1
# Here is how the map from this configuration looks:



# You can play with the maps (stored in config file "maps/test_config.txt") by manually controlling the robot by executing the follwing command:

# python3 mp3.py --human --map Test1 --config maps/test_config.txt
# Feel free to modify the config files to do more self tests by adding test maps of your own.
# Once the window pops up, you can manipulate the alien using the following keys:

# w / a / s / d: move the alien up / left / down/ right
# q / e: switch between horizontal, ball, and vertical forms of the alien. "q" will cycle backwards (Vertical -> Ball -> Horizontal) and "e" will cycle forwards (Horizontal -> Ball -> Vertical
# While implementing your geometry.py file, you can also use this mode to manually verify your solution, as the alien should turn red when touching an obstacle or out of bounds, and should turn green when validly completing the course, as shown in the initial figures.



# The first part of this MP is to extend your A* code from MP2 to handle 3D mazes. Your A* code must be able to handle the possibility that there are multiple goals. However, unlike MP 2, your code does not have to touch all the goals. It should consider the maze "solved" when it reaches the first goal. Your new A* code should be added to this file:
# search.py

# astar(maze): returns optimal path in a list of MazeState objects, which contains start and objectives. If no path found, return None.
# backtrack(visited_states, current_state): this should be similar to your MP 2 implementation.
# To implement search, you will also need to finish the implementation of MazeState in state.py. It might help to review some of the states you made in the previous MP. Complete all of the TODOs to finish the implementation. Note that unlike the previous MP, we do not include the MST in the heuristic, because we only need to reach one goal. Given the new objective, our old heuristic would not be consistent and admissible.

# Run part 1 by using the following command

# python3 part1.py mazes/small-3d
# You can control the agent with keyboard inputs as usual by using arrow keys to navigate on the same level and 'u' and 'd' to move up/down a level

# python3 part1.py mazes/small-3d --human
# NOTE: unlike MP2, in this MP, the maze might not have a path! So be sure to handle this case properly and return None! Also notice that the astar function now takes an extra argument ispart1. You should pass this argument as it is when you call getNeighbors() Don't change it as it will mess up the tests.


def search(maze, searchMethod):
    return {
        "astar": astar,
    }.get(searchMethod, [])(maze)

def astar(maze, ispart1=False):
    """
    This function returns an optimal path in a list, which contains the start and objective.

    @param maze: Maze instance from maze.py
    @param ispart1: pass this variable when you use functions such as getNeighbors and isObjective. DO NOT MODIFY THIS
    @return: a path in the form of a list of MazeState objects
    """
    # Returns the start position as a tuple of (row, col, level)
    start = maze.getStart()
    # initialize the frontier
    frontier = []
    # initialize the visited states
    visited_states = []
    # add the start MazeState to the frontier
    heapq.heappush(frontier, start)
    # add the start state to the visited states
    visited_states.append(start)
    # cost_so_far is a dictionary that maps a state to its cost
    cost_so_far = {}
    # set the cost of the start state to 0
    cost_so_far[start] = 0
    # parent is a dictionary that maps a state to its parent
    parent = {}
    # set the parent of the start state to None
    parent[start] = None
    # while the length of the frontier is greater than 0
    while len(frontier) > 0:
        # pop the state with the lowest cost from the frontier
        current_state = heapq.heappop(frontier)
        # if the current state is an objective
        if current_state.is_goal():
            # return the path
            # print('Visited: ', visited_states, 'CUREENT: ', current_state)
            return backtrack(visited_states, current_state, parent)
            # return the path from the start to the current state
        # for each neighbor of the current state
        neighbors = current_state.get_neighbors(ispart1)
        for neighbor in neighbors:
            # if the neighbor has not been visited
            if neighbor not in visited_states:
                # add the neighbor to the frontier
                heapq.heappush(frontier, neighbor)
                # add the neighbor to the visited states
                visited_states.append(neighbor)
                # set the cost of the neighbor to the cost of the current state + 1
                cost_so_far[neighbor] = cost_so_far[current_state] + 1
                # set the parent of the neighbor to the current state
                parent[neighbor] = current_state
    # if no path is found, return None
    return None




# This is the same as backtrack from MP2
def backtrack(visited_states, current_state, parent):
    """
    This function returns a path in the form of a list of MazeState objects

    @param visited_states: a list of MazeState objects
    @param current_state: a MazeState object
    @return: a path in the form of a list of MazeState objects that leads from the start to the current state
    """
#     if path_found:
    #     while target_node is not None:
    #         path.append(target_node)
    #         target_node = parents[target_node]
    #     path.reverse()
    # return (path, iteration)
    path = []  
    while current_state is not None:
        path.append(current_state)
        current_state = parent[current_state]
    path.reverse()
    return path