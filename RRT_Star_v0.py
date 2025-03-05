import numpy as np
import matplotlib.pyplot as plt
import math
import random
import csv

def read_map_from_file(filename):
    '''
    This functions reads a csv file describing a map and returns the map data
    Inputs:
        - filename (string): name of the file to read
    Outputs:
        - map (tuple): A map is a tuple of the form (grid_size, start_pos, goal_pos, [obstacles])
            grid_size is an tuple (length, height) representing the size of the map
            start_pos is a tuple (x, y) representing the x,y coordinates of the start position
            goal_pos is a tuple (x, y) representing the x,y coordinate of the goal position
            obstacles is a list of tuples. Each tuple represents a single  circular obstacle and is of the form (x, y, radius).
                x is an integer representing the x coordinate of the obstacle
                y is an integer representing the y coordinate of the obstacle
                radius is an integer representing the radius of the obstacle
    '''

    #Your code goes here
    # CSV file -> list
    with open(filename,'r') as file:
        # Convert csv file into an CSV object
        csv_temp = csv.reader(file)
        
        # Convert CSV object into list
        data = list(csv_temp)

    # Extract map variables
    # Extract values
    x_Dimension = int(data[0][0])
    y_Dimension = int(data[0][1])
    start_x = int(data[1][0])
    start_y = int(data[1][1])
    goal_x = int(data[2][0])
    goal_y = int(data[2][1])
    # Combine into turple
    grid_size = (x_Dimension, y_Dimension)
    start_pos = (start_x,start_y)
    goal_pos = (goal_x,goal_y)
    
    # Extract obstacles variables
    obstacles = [] # A list where all obstacles stores
    for line in data[3:]: # Loop every obstacles until end of list
        # Extract values
        obstacle_x = int(line[0])
        obstacle_y = int(line[1])
        obstacle_r = int(line[2])
        # Combine into turple
        obstacle = (obstacle_x, obstacle_y, obstacle_r)
        # Append into big obstacles list
        obstacles.append(obstacle)
    
    # Combine everything together
    map = (grid_size, start_pos, goal_pos, obstacles)

    # Return map
    return map



class Node:
    def __init__(self, x:float, y:float, parent=None):
        """
        Minimum unit of a graph, node stores here

        Args:
            x: position of node on x-axis
            y: position of node on y-axis
            parent: stores previous node, default None
            
        Public Variables and Methods:
        self.x
        self.y
        self.parent
        euclideanDistance(self, node_2)->float
        """
        self.x = x
        self.y = y
        self.parent = parent
    
    def euclideanDistance(self, node_2)->float:
        """
        Calculating the straight line distance between self and imported node

        Args:
            node_2: 2nd node to compare, datatype class Node
            
        Returns:
            length in float
        """
        return math.sqrt((self.x - node_2.x)**2 + (self.y - node_2.y)**2)



class Obstacle:
    def __init__(self):
        self.type = None
        self.position = None # Use node
        self.radius = None
    def isBlocked(self):
        pass



class Map:
    def __init__(self,map:tuple,start:tuple,goal:tuple,obstacles:list=[]):
        """
        Stores everything about map, including map size, start and goal points, list of obstacles

        Args:
            map: Map size in tuple format: (x_size,y_size)
            start: Position of start point in class Node
            goal: Position of goal point in class Node
            obstacles: A list of objects in class Obstacle, defaule None
            
        Public Variables and Methods:
            self.x_size
            self.y_size
            self.start
            self.goal
            self.obstacles
            plot(self)
        """
        self.x_size = map[0]
        self.y_size = map[1]
        self.start = Node(start[0],start[1])
        self.goal = Node(goal[0],goal[1])
        self.obstacles = obstacles

    def plot(self):
        """
        Plot the map with obstacles, no path
        """
        pass



class RRT:
    def __init__(self, map:Map, nodes:list=[], max_step:float=10, goal_radius:float=10, exploration_bias:float=0, max_iteration:int=100, enable_star:bool=True):
        # Map
        self.map = map
        # Nodes
        self.nodes = nodes
        # Config
        self.max_step = max_step
        self.goal_radius = goal_radius
        self.exploration_bias = exploration_bias
        self.max_iteration = max_iteration
        self.enable_star = enable_star
    
    def newNode(self)->Node:
        """
        Create a new node randomly, may add bias later?????

        Returns:
            Node with random x and y
        """
        x = random.uniform(0, self.map.x_length)
        y = random.uniform(0, self.map.y_length)
        return Node(x, y)
    
    def nearestNode(self):
        """
        Calculating the straight line distance between self and imported node

        Args:
            node_2: 2nd node to compare, datatype class Node
            
        Returns:
            length in float
        """
        # Return closest node
        pass
    def nearNodes(self):
        """
        Calculating the straight line distance between self and imported node

        Args:
            node_2: 2nd node to compare, datatype class Node
            
        Returns:
            length in float
        """
        # Return nodes inside range
        pass
    def isCollision(self):
        """
        Calculating the straight line distance between self and imported node

        Args:
            node_2: 2nd node to compare, datatype class Node
            
        Returns:
            length in float
        """
        # Check collision
        pass
    def buildTree(self):
        """
        Calculating the straight line distance between self and imported node

        Args:
            node_2: 2nd node to compare, datatype class Node
            
        Returns:
            length in float
        """
        # Create tree, main function
        pass
    def plot(self):
        """
        Calculating the straight line distance between self and imported node

        Args:
            node_2: 2nd node to compare, datatype class Node
            
        Returns:
            length in float
        """
        # Plot graph while going
        pass






"""
# Example Usage
maps = ["map1.csv","map2.csv","map3.csv"]
for a in maps:
    map_file = read_map_from_file(a)
    map_size = map_file[0]
    start = map_file[2]
    goal = map_file[1]
    obstacles = map_file[3]
    step_size = 10
    num_points = 800

    path, edges = rrt(map_size, start, goal, obstacles, step_size, num_points)
    plot_rrt(map_size, start, goal, obstacles, path, edges) 
"""