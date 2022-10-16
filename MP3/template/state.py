import copy

# use helper functions from utils.py and Alien.py
# and State and A* code to be able to search in three dimensions

from itertools import count
# NOTE: using this global index means that if we solve multiple 
#       searches consecutively the index doesn't reset to 0... this is fine
global_index = count()

# TODO: implement this method
def manhattan(a, b):
    """
    Computes the manhattan distance
    @param a: a length-3 state tuple (x, y, shape)
    @param b: a length-3 state tuple
    @return: the manhattan distance between a and b
    """
    # print(a, b)
    # transitioning from shape to shape also counts in the distance, so you would have to include the difference in shape idx in the manhattan distance
    return abs(a[0] - b[0]) + abs(a[1] - b[1]) + abs(a[2] - b[2])

        

from abc import ABC, abstractmethod
class AbstractState(ABC):
    def __init__(self, state, goal, dist_from_start=0, use_heuristic=True):
        self.state = state
        self.goal = goal
        # we tiebreak based on the order that the state was created/found
        self.tiebreak_idx = next(global_index)
        # dist_from_start is classically called "g" when describing A*, i.e., f = g + h
        self.dist_from_start = dist_from_start
        self.use_heuristic = use_heuristic
        if use_heuristic:
            self.h = self.compute_heuristic()
        else:
            self.h = 0

    # To search a space we will iteratively call self.get_neighbors()
    # Return a list of State objects
    @abstractmethod
    def get_neighbors(self):
        pass
    
    # Return True if the state is the goal
    @abstractmethod
    def is_goal(self):
        pass
    
    # A* requires we compute a heuristic from each state
    # compute_heuristic should depend on self.state and self.goal
    # Return a float
    @abstractmethod
    def compute_heuristic(self):
        pass
    
    # The "less than" method ensures that states are comparable
    #   meaning we can place them in a priority queue
    # You should compare states based on f = g + h = self.dist_from_start + self.h
    # Return True if self is less than other
    @abstractmethod
    def __lt__(self, other):
        # NOTE: if the two states (self and other) have the same f value, tiebreak using tiebreak_idx as below
        if self.tiebreak_idx < other.tiebreak_idx:
            return True

    # __hash__ method allow us to keep track of which 
    #   states have been visited before in a dictionary
    # You should hash states based on self.state (and sometimes self.goal, if it can change)
    # Return a float
    @abstractmethod
    def __hash__(self):
        pass
    # __eq__ gets called during hashing collisions, without it Python checks object equality
    @abstractmethod
    def __eq__(self, other):
        pass


# State: a length 3 list indicating the current location in the grid and the shape
# Goal: a tuple of locations in the grid that have not yet been reached
#   NOTE: it is more efficient to store this as a binary string...
# maze: a maze object (deals with checking collision with walls...)
# mst_cache: You will not use mst_cache for this MP. reference to a dictionary which caches a set of goal locations to their MST value
class MazeState(AbstractState):
    def __init__(self, state, goal, dist_from_start, maze, mst_cache={}, use_heuristic=True):
        # NOTE: it is technically more efficient to store both the mst_cache and the maze_neighbors functions globally, 
        #       or in the search function, but this is ultimately not very inefficient memory-wise
        self.maze = maze
        self.mst_cache = mst_cache # DO NOT USE
        self.maze_neighbors = maze.getNeighbors
        super().__init__(state, goal, dist_from_start, use_heuristic)
        
    # TODO: implement this method
    # Unlike MP 2, we do not need to remove goals, because we only want to reach one of the goals
    def get_neighbors(self, ispart1=False):
        nbr_states = []
        # We provide you with a method for getting a list of neighbors of a state
        # that uses the Maze's getNeighbors function.
        neighboring_locs = self.maze_neighbors(*self.state, part1=ispart1)
        # print('NEIGHBORS: ', neighboring_locs)
        for loc in neighboring_locs:
            nbr_states.append(MazeState(loc, self.goal, self.dist_from_start + 1, self.maze, self.mst_cache, self.use_heuristic))
        return nbr_states
        


    # TODO: implement this method
    # so the goal states have the same form.
    # remember 3D state
    def is_goal(self):
        """
        Returns True if the state is the goal
        """
        # print('SELF STATE', self.state)
        return self.state in self.goal

        # for goal in self.goal:
        #     if self.state == goal:
        #         return True
        # return False


    # TODO: implement these methods __hash__ AND __eq__
    def __hash__(self):
        return hash(self.state)

    def __eq__(self, other):
        # print('SELF:', self.state, 'Other: ', other)
        # SELF: (1, 1, 0) Other:  (1, 1, 2)
        return self.state == other.state
        
    # TODO: implement this method
    def compute_heuristic(self):
        # the third tuple corresponds to what the shape is (0 is horizontal, 1 is ball, 2 is vertical)
        # the heuristic is the manhattan distance from the current state to the goal state
        # print('STATE', self.state)
        return manhattan(self.goal[0], self.state)

    # TODO: implement this method. It should be similar to MP 2
    def __lt__(self, other):
        """
        Compares two states
        @param other: the other state to compare to
        @return: True if self is less than other
        """
        if self.dist_from_start + self.h < other.dist_from_start + other.h:
            return True
        elif self.dist_from_start + self.h == other.dist_from_start + other.h:
            if self.tiebreak_idx < other.tiebreak_idx:
                return True
        return False

    # str and repr just make output more readable when your print out states
    def __str__(self):
        return str(self.state) + ", goals=" + str(self.goal)
    def __repr__(self):
        return str(self.state) + ", goals=" + str(self.goal)
