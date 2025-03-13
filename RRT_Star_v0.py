import matplotlib.pyplot as plt
import math
import random



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
        plot(self,ax,style:str)
        plotParentPath(self,ax,style:str)
        __str__(self) -> str
        """
        self.x = x
        self.y = y
        self.parent = parent
    
    def euclideanDistance(self, node_2) -> float:
        """
        Calculating the straight line distance between self and imported node

        Args:
            node_2: 2nd node to calculate, datatype class Node
            
        Returns:
            Length in float
        """
        return math.sqrt((self.x - node_2.x)**2 + (self.y - node_2.y)**2)

    def plot(self,ax,style:str):
        """
        Plot node itself with passed in style

        Args:
            ax: The figure
            style: Style for plotting the node
        """
        ax.plot(self.x, self.y, style)

    def plotParentPath(self,ax,style:str):
        """
        Plot a path between node itself and its parent node with given style

        Args:
            ax: The figure
            style: Style for plotting the path
        """
        ax.plot([self.x, self.parent.x],[self.y, self.parent.y], style)

    def __str__(self) -> str:
        """
        Print node in "(x,y)" format

        Returns:
            String of it's own value
        """
        return str(f"({self.x},{self.y})")



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
            plot(self.ax)
        """
        self.type = 'circle' # Identify the type of obstacle
        self.center = center
        self.radius = radius
    
    def isBlocked(self,node_1:Node,node_2:Node=None) -> bool:
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

        # Return True if either of node 1 or node 2 inside circle
        if node_1_blocked or node_2_blocked:
            return True
        
        # Check line between node 1 and 2 touches or intersects or outside
        # Rearrange line equation ax + by + c = 0 and initialize variables
        a = node_2.y - node_1.y
        b = node_1.x - node_2.x
        c = ((node_2.x - node_1.x) * node_1.y - (node_2.y - node_1.y) * node_1.x)
        x = self.center.x
        y = self.center.y
        
        # Calculate closest distance to circle center using d=abs(ax+by+c)/((a^2+b^2)^1/2)
        top = (abs(a * x + b * y + c))
        bottom = math.sqrt(a * a + b * b)
        try:
            distance = (top / bottom)
        except ZeroDivisionError:
            # Abandon node if zero division error
            return True
        
        # Return Result
        if (self.radius >= distance): return True
        else: return False
    
    def plot(self,ax):
        """
        Plot circle on graph

        Args:
            ax: The figure
            style: Style for plotting the path
        """
        ax.add_patch(plt.Circle((self.center.x, self.center.y), self.radius, color='red'))



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

    def plot(self,ax):
        pass



class Map:
    def __init__(self,map:tuple,start:Node,goal:Node,obstacles:list[ObstacleCircle,ObstaclePolygon]=[]):
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
            plot(self,ax)
        """
        self.x_size = map[0]
        self.y_size = map[1]
        self.start = start
        self.goal = goal
        self.obstacles = obstacles

    def plot(self,ax):
        """
        Plot the map with obstacles, no path
        """
        # Set figure aspect ration to map aspect ratio
        ax.set_aspect(self.y_size/self.x_size)

        # Restrict total area
        ax.set_xlim(0,self.x_size)
        ax.set_ylim(0,self.y_size)

        # Plot start and goal point
        self.start.plot(ax,'r*')
        self.goal.plot(ax,'b*')

        # Plot obstacle
        for obstacle in self.obstacles:
            obstacle.plot(ax)



class RRT:
    def __init__(self, map:Map, \
                nodes:list[Node]=[], \
                int_node:bool=True, exploration_bias:float=0, max_retry:int = 20, \
                force_step:bool=False, step_ratio:float = 0.1, max_step:float=10, goal_radius:float=10, \
                max_iteration:int=200, star_iteration:int=0, \
                animation_duration:float=0.02):
        """
        A place where everything combines together
        
        Args:
            map: Map in class Map
            nodes: A list of existing tree nodes in class Node; Default empty list
            int_node: Indentify whether rounding to integer is needed; Default True
            exploration_bias: The probability of set goal as random point, speed up sometimes; Default 0
            max_retry: Max tries for find a non-exist node; Default 20
            force_step: True if every path has to be max_step length; Default True
            step_ratio: A ratio identify how long max step should be depending on map, set 0 to disable; Default 0.1
            max_step: Maximum length of each step; Defaule 10
            goal_radius: Any node within this range of goal will consider reach goal, set 0 if same as max step; Default 10
            max_iteration: Max iterations before reach a goal; Default 200
            star_iteration: Total iterations for RRT* algorithm, set 0 to disable RRT*; Default 0
            animation_duration: Duration for each frame; Default 0.02
            
        Public Variables and Methods:
        self.x
        self.y
        self.parent
        euclideanDistance(self, node_2:Node)->float
        plot(self,ax,style:str)
        plotParentPath(self,ax,style:str)
        __str__(self) -> str

        """
        # Map
        self.map = map
        
        # Nodes
        self.nodes = nodes
        self.nodes.append(self.map.start) # Add start into node list
        
        # Config - Node
        self.int_node = int_node
        self.exploration_bias = exploration_bias # Range 0-1
        self.max_retry = max_retry 

        # Config - Step
        self.force_step = force_step
        # Apply ratio to set a variable step size
        if step_ratio:
            self.max_step = ((self.map.x_size + self.map.y_size) / 2 * step_ratio)
        # Use input step size
        else:
            self.max_step = max_step
        # Apply ratio to set a variable step size
        if step_ratio:
            self.max_step = ((self.map.x_size + self.map.y_size) / 2 * step_ratio)
        # Use input step size
        else:
            self.max_step = max_step
        if goal_radius:
            self.goal_radius = goal_radius
        # Use max_step as goal radius
        else:
            self.goal_radius = self.max_step
        
        # Config - Iteration
        self.current_iteration = 0 # Define current iteration
        self.max_iteration = max_iteration # Max iteration before finding a path
        self.star_iteration = star_iteration # Number of star iteration, set 0 to disable
        
        # Config - Plotting
        self.animation_duration = animation_duration
    
    def newNode(self) -> Node:
        """
        Create a new node randomly, in integers or decimal numbers.\n
        To apply bias, set self.exploration_bias, so there will be certain chance using goal as random point.\n
        Applying bias may reduce required nodes to reach goal in some cases, may also become worth if there are obstacles towards goal direction.

        Returns:
            Node with random x and y
        """
        # Apply bias at set lucky rate
        if random.random() <= self.exploration_bias:
            return self.map.goal
        
        # Generate new node randomly in map
        # If int_node enable, random nodes will be integers only
        if self.int_node:
            # Generate random integer node
            x = random.randint(0, self.map.x_size)
            y = random.randint(0, self.map.y_size)
            
        # Generate random numbers with  decimal points
        else:
            # Generate random node
            x = random.uniform(0, self.map.x_size)
            y = random.uniform(0, self.map.y_size)
        
        return Node(x, y)
    
    def nearestNode(self) -> Node:
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
            distance = self.random_node.euclideanDistance(node)
            distance_list.append(distance)
        
        # Return the node has minimum distance
        minimum_index = distance_list.index(min(distance_list))
        self.nearest_node = self.nodes[minimum_index]
        return self.nearest_node

    def placeNode(self) -> Node:
        """
        Place new node either any length within step, or force to step length

        Returns:
            Node which fit maximum step size requirement
        """
        # Find distance between 2 points
        distance = self.random_node.euclideanDistance(self.nearest_node)

        # If distance <= max step and not force to use fixed step length
        if ((distance <= self.max_step) and (not self.force_step)):
            return self.random_node

        # Limit the step to within step size if too long
        # Limit using ratio
        x_new = self.nearest_node.x + (self.max_step / distance) * (self.random_node.x - self.nearest_node.x)
        y_new = self.nearest_node.y + (self.max_step / distance) * (self.random_node.y - self.nearest_node.y)

        # Check if round needed
        if self.int_node:
            return Node(round(x_new),round(y_new))
        else:
            return Node(x_new,y_new)

    def ifObstacle(self, node_1:Node, node_2:Node) -> bool:
        """
        Loop all obstacles to check if new line between node 1 and node 2 has blocked
        If index out of range, return True

        Args:
            node_1: Node 1 to check
            node_2: Node 2 to check
            
        Returns:
            True if blocked; False if not blocked
        """
        # Check if index in range
        for node in (node_1,node_2):
            if (node.x < 0) or (node.x > self.map.x_size) or (node.y < 0) or (node.y > self.map.y_size):
                # Reject any index out of range due to enlarge
                return True

        # Loop each obstacle
        for obstacle in self.map.obstacles:
            # Check if blocked
            if obstacle.isBlocked(node_1, node_2):
                # Return True if blocked by any obstacle
                return True
        
        # Return False if not blocked at all
        return False

    def ifExistNode(self, node_to_check:Node) -> bool:
        """
        Check if one node has already exist in tree

        Args:
            node_to_check: Check if this node already exist
            
        Returns:
            True if exist; False if not exist
        """
        # Loop all the nodes to find if has same value
        for node in self.nodes:
            if (node.x == node_to_check.x) and (node.y == node_to_check.y):
                return True
        return False

    def connectNode(self, ax, old_node:Node, new_node:Node):
        """
        Assign parent to new node and store it\n
        Remain space for RRT* algorithm, which may reconnnect nodes around

        Args:
            ax: The figure
            old_node: Parent node put into new node
            new_node: Add parent and store this new node
        """
        # For RRT Star only
        if self.star_iteration:
            # Remained for RRT*
            # 1*. Place node , distance/max step, choose shorter
            # 2*. Check nodes around, reconnect if required
            pass
        
        # For RRT only
        else:
            # Assign parent to new node
            new_node.parent = old_node
            # Return new node with parent
            self.nodes.append(new_node)
            
            # Plot new node and new path
            self.nodes[-1].plot(ax, 'b.')
            self.nodes[-1].plotParentPath(ax,'k-')

    def reachGoal(self) -> bool:
        """
        Check if the goal is reachable
            
        Returns:
            True if reach goal; False if not reach goal or requirement failed
        """
        # Check if last node is within goal range
        if self.new_node.euclideanDistance(self.map.goal) < self.goal_radius:
            # Check connection to goal has obstacle
            if not self.ifObstacle(self.map.goal, self.new_node):
                return True
        # Return False if requirement not fit
        return False

    def getPath(self) -> list[(tuple,tuple)]:
        """
        Find the final success path

        Returns:
            A list of tuples with success path
        """
        # Get result path if success
        current_node = self.nodes[-1]
        path = []
        while current_node.parent is not None:
            path.append(current_node)
            current_node = current_node.parent
        return path

    def buildTree(self) -> int:
        """
        Steps to build a tree:
        1. Generate new node (done)
        2. Find nearest node (done)
        3. Check distance, if >stepsize, limit to stepsize + rounding
        4. If no obstacle, place it, or back to 1
        5. Check if reach goal, if yes, finish
        6. Check if near goal, if yes, go 3

        For RRT* only
        5*. If RRT*, place node , distance/max step, choose shorter
        6*. Check nodes around, reconnect if required
        
        Returns:
            Number of iterations
        """
        # Initialize plot
        plt.ion() # Keep program running while plotting
        fig = plt.figure()
        ax = fig.add_subplot(111)
        self.map.plot(ax)
        plt.show()

        # Some loop
        while True:
            # Initialize variables
            self.random_node = None          
            self.nearest_node = None
            self.new_node = None
            counter = 0
            # Increment current iteration
            self.current_iteration += 1
            ###DEBUG print(f"Iteration: {self.current_iteration}")

            # Print title message
            plt.title(f"Iteration: {self.current_iteration}, Step: {self.max_step}, Bias: {self.exploration_bias}, Fix Step: {self.force_step}")

            # 1. Generate a random node
            self.random_node = self.newNode()
            
            # Regenerate if node exist  ### Leave into generate new node part
            while self.ifExistNode(self.random_node):
                counter += 1
                self.random_node = self.newNode()
                if counter >= self.max_retry:
                    # Terminate if too many tries on generating new node ###
                    flag = False
                    break
            
            # Plot random node
            self.random_node.plot(ax,'+')

            # 2. Find nearest exist node
            self.nearest_node = self.nearestNode()

            # 3. Get value for the new node
            self.new_node = self.placeNode()
            
            # 4. If new node does not exist, and not obstacled, connect node and add to list
            if (not self.ifExistNode(self.new_node)) and (not self.ifObstacle(self.nearest_node, self.new_node)):
                self.connectNode(ax,self.nearest_node, self.new_node)
            
                # 5. Check if reach goal, if yes, finish reachGoal(self)
                if self.reachGoal():
                    self.connectNode(ax,self.nodes[-1], self.map.goal)

                    ###print("Solution found")
                    # Connect goal if in range
                    ### Call output path
                    path = self.getPath()

                    # Plot in cyan
                    for node in path:
                        node.plotParentPath(ax,'c-')

                    flag = True # Success
                    plt.title(f"Valid Path! Iteration: {self.current_iteration}, Step: {self.max_step}, Bias: {self.exploration_bias}, Fix Step: {self.force_step}")
                    break
                
            # Check if reach maximum iteration
            if self.current_iteration >= self.max_iteration:
                # Break if reach max iteration
                flag =  False
                plt.title(f"Max Iteration Reached! Iteration: {self.current_iteration}, Step: {self.max_step}, Bias: {self.exploration_bias}, Fix Step: {self.force_step}")
                break

            # Plot the graph with slightly delay
            plt.show()
            plt.pause(self.animation_duration)
        
        # Show final plot
        plt.ioff()
        plt.show()
        # Return result
        return self.current_iteration



# Test run
map1 = Map((100,100),Node(5,5),Node(95, 95),[ObstacleCircle(Node(10, 10), 2),ObstacleCircle(Node(30, 10), 2),ObstacleCircle(Node(50, 10), 7)])
map2 = Map((150,150),Node(5,5),Node(95, 95),[ObstacleCircle(Node(10, 90), 10),ObstacleCircle(Node(30, 30), 20),ObstacleCircle(Node(50, 10), 7)])
map3 = Map((50,50),Node(45,48),Node(5, 5),[ObstacleCircle(Node(25,10), 20),ObstacleCircle(Node(25,40), 20)])

rrt = RRT(map2,exploration_bias=0,max_iteration=200,force_step=True)

with open('data-25.txt','a') as file:
    file.writelines(f"{rrt.buildTree()}\n")