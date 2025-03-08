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
    def __init__(self,map:tuple,start:tuple,goal:tuple,obstacles:list[ObstacleCircle,ObstaclePolygon]=[]):
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
    def __init__(self, map:Map, nodes:list[Node]=[], int_node:bool=True, exploration_bias:float=0, max_step:float=10, goal_radius:float=10, max_iteration:int=100, star_iteration:int=0):
        # Map
        self.map = map
        # Nodes
        self.nodes = nodes.append(self.map.start) # Add start into node list
        # Config
        self.int_node = int_node
        self.exploration_bias = exploration_bias
        self.max_step = max_step
        self.goal_radius = goal_radius
        self.current_iteration = 0 # Define current iteration
        self.max_iteration = max_iteration # Max iteration before finding a path
        self.star_iteration = star_iteration # Number of star iteration, set 0 to disable
    
    def newNode(self)->Node:
        """
        Create a new node randomly, bias may apply. 
        May choose to round to integer nodes.

        Returns:
            Node with random x and y
        """
        # Generate new node biased towards goal
        if self.exploration_bias:
            pass
        
        # Generate new node randomly in map
        else:
            # If int_node enable, nodes will be integers only
            if self.int_node:
                # Generate random integer node
                x = random.randint(0, self.map.x_size)
                y = random.randint(0, self.map.y_size)
            
            # Generate random numbers with  decimal points
            else:
                # Generate random node
                x = random.uniform(0, self.map.x_size)
                y = random.uniform(0, self.map.y_size)
        
        self.random_node = Node(x, y)
    
    def nearestNode(self, new_node:Node)->Node:
        """
        Find nearest node in exist node list 

        Returns:
            Node which is closest to new random node
        """
        # Initialize an empty distance list
        distance_list = []

        # Find distance to all existing nodes
        for node in self.nodes:
            # And add this into distance list
            distance_list.append(new_node.euclideanDistance(node))
        
        # Return the node has minimum distance
        return self.nodes[distance_list.index(min(distance_list))]

    def placeNode(self, random_node:Node, nearest_node:Node)->Node:
        """
        Place new node either within step, or limit it to step size

        Args:
            random_node: New randomaly generated node to place or reference
            nearest_node: Parent of new node
            
        Returns:
            Node which fit maximum step size requirement
        """
        # Find distance between 2 points
        distance = random_node.euclideanDistance(nearest_node)

        # If distance < max step, return that node
        if distance <= self.max_step:
            return random_node

        # Limit the step to within step size if too long
        # Limit using ratio
        x_new = nearest_node.x + (self.max_step / distance) * (random_node.x - nearest_node.x)
        y_new = nearest_node.y + (self.max_step / distance) * (random_node.y - nearest_node.y)

        # Check if round needed
        if self.int_node:
            return Node(round(x_new),round(y_new))
        else:
            return Node(x_new,y_new)

    def ifObstacle(self, nearest_node:Node, new_node:Node)->bool:
        """
        Loop all obstacles to check if new line has blocked

        Args:
            nearest_node: Parent of new node
            new_node: New generated node
            
        Returns:
            True if blocked; False if not blocked
        """
        # Loop each obstacle
        for obstacle in self.map.obstacles:
            # Check if blocked
            if obstacle.isBlocked(nearest_node, new_node):
                # Return True if blocked by any obstacle
                return True
        # Return False if not blocked at all
        return True

    def ifExistNode(self, new_node:Node)->bool:
        return any(node.x == new_node.x and node.y == new_node.y for node in self.nodes)

    def connectNode(self, nearest_node:Node, new_node:Node):
        # For RRT Star
        if self.star_iteration:
            # Remained for RRT*
            # 5*. If RRT*, place node , distance/max step, choose shorter
            # 6*. Check nodes around, reconnect if required
            pass
        
        else:
            # Assign parent to new node
            new_node.parent = nearest_node
            # Return new node with parent
            self.nodes.append(new_node)

    def reachGoal(self):
        # Check if reach goal
        if self.new_node.euclideanDistance(self.map.goal) < self.goal_radius:
            return True
        else:
            return False

    def getPath(self):
        pass
        # Get result path if success

    def buildTree(self):
        """
        1. Generate new node (done)
        2. Find nearest node (done)
        3. Check distance, if >stepsize, limit to stepsize + rounding
        4. If no obstacle, place it, or back to 1
        5*. If RRT*, place node , distance/max step, choose shorter
        6*. Check nodes around, reconnect if required
        7. Check if reach goal, if yes, finish
        8. Check if near goal, if yes, go 3
        """
        # Some loop
        while True:
            # Initialize variables
            self.random_node = None          
            self.nearest_node = None
            self.new_node = None
            # Increment current iteration
            self.current_iteration += 1

            # 1. Generate a random node
            self.newNode()
            # Regenerate if node exist  ### Leave into generate new node part
            while self.random_node in self.nodes:
                self.newNode()

            # 2. Find nearest exist node
            nearest_node = self.newNode(self.random_node)

            # 3. Place the new node
            self.new_node = self.placeNode(self.random_node, nearest_node)
            
            # 4. If new node does not exist, and not obstacled, connect node and add to list
            if not (self.ifExistNode(self.new_node) or self.ifObstacle(nearest_node, self.new_node)):
                self.connectNode(nearest_node, self.new_node)
            
            # 5. Check if reach goal, if yes, finish reachGoal(self)
            if self.reachGoal():
                # Check connection to goal has obstacle
                if self.ifObstacle(self.new_node, self.map.goal):
                    # Connect goal if in range
                    self.connectNode(self.new_node, self.map.goal)
            
            # Check if reach maximum iteration
            if self.current_iteration >= self.max_iteration:
                # Break if reach max iteration
                break

    def plotNode(self):
        """
        Calculating the straight line distance between self and imported node

        Args:
            node_2: 2nd node to compare, datatype class Node
            
        Returns:
            length in float
        """
        # Plot graph while going
        pass

    def plotPath(self):
        """
        Calculating the straight line distance between self and imported node

        Args:
            node_2: 2nd node to compare, datatype class Node
            
        Returns:
            length in float
        """
        # Plot graph while going
        pass

