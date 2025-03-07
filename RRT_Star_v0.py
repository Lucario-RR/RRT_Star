import numpy as np
import matplotlib.pyplot as plt
import math
import random
import csv



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
        euclideanDistance(self, node_2:Node)->float
        """
        self.x = x
        self.y = y
        self.parent = parent
    
    def euclideanDistance(self, node_2)->float:
        """
        Calculating the straight line distance between self and imported node

        Args:
            node_2: 2nd node to calculate, datatype class Node
            
        Returns:
            Length in float
        """
        return math.sqrt((self.x - node_2.x)**2 + (self.y - node_2.y)**2)



class ObstacleCircle:
    def __init__(self, center:Node, radius:float):
        """
        Stores circular obstacles

        Args:
            center: Center of circle in class Node
            radius: Radius in Float
            
        Public Variables and Methods:
            self.type
            self.center
            self.radius
            isBlocked(self,node_1:Node,node_2:Node=None)->bool
        """
        self.type = 'circle' # Identify the type of obstacle
        self.center = center
        self.radius = radius
    
    def isBlocked(self,node_1:Node,node_2:Node=None)->bool:
        """
        Check if 1 node or line between 2 nodes is inside this circular object

        Args:
            node_1: 1st node to check, datatype class Node
            node_2: 2nd node to form a line with node_1, datatype class Node
            
        Returns:
            True if blocked; False if not blocked
        """
        # Check if node 1 is inside circle or not
        node_1_blocked = (self.center.euclideanDistance(node_1) <= self.radius)
        
        # Return if only 1 node passed in
        if node_2 is None:
            return node_1_blocked

        # Check if node 2 is inside circle or not
        node_2_blocked = (self.center.euclideanDistance(node_2) <= self.radius)

        # Return False if either of node 1 or node 2 inside circle
        if node_1_blocked or node_2_blocked:
            return False
        
        # Check line between node 1 and 2 touches or intersects or outside
        # Rearrange line equation ax + by + c = 0 and initialize variables
        a = node_2.y - node_1.y
        b = node_1.x - node_2.y
        c = ((node_2.x - node_1.x) * node_1.y - (node_2.y - node_1.y) * node_1.x)
        x = self.center.x
        y = self.center.y
        # Calculate closest distance to circle center using d=abs(ax+by+c)/((a^2+b^2)^1/2)
        distance = ((abs(a * x + b * y + c)) /
			        math.sqrt(a * a + b * b))
        
        # Return Result
        if (self.radius < distance): return True
        else: return False



class ObstaclePolygon:
    def __init__(self,type:str,positions:list[Node]=[]):
        """
        Stores circular obstacles

        Args:
            type: Type of this object, e.g triangle, rectangle...
            positions: List of corner in class Node
            
        Public Variables and Methods:
            self.type
            self.positions
            isBlocked(self)->bool
        """
        self.type = type
        self.positions = positions # List of corners in class Node
    
    def isBlocked(self,node_1:Node,node_2:Node=None)->bool:
        """
        Check if 1 node or line between 2 nodes is inside this obstacle

        Args:
            node_1: 1st node to check, datatype class Node
            node_2: 2nd node to form a line with node_1, datatype class Node
            
        Returns:
            True if blocked; False if not blocked
        """
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