# Standard Algorithm Implementation
# Sampling-based Algorithms PRM
import matplotlib.pyplot as plt
import numpy as np
import networkx as nx

import math 
from numpy import random 
from scipy import spatial 


# Class for PRM
class PRM:
    # Constructor
    def __init__(self, map_array):
        self.map_array = map_array            # map array, 1->free, 0->obstacle
        self.size_row = map_array.shape[0]    # map size
        self.size_col = map_array.shape[1]    # map size

        self.samples = []                     # list of sampled points
        self.graph = nx.Graph()               # constructed graph
        self.path = []                        # list of nodes of the found path


    def check_collision(self, p1, p2):
        '''Check if the path between two points collide with obstacles
        arguments:
            p1 - point 1, [row, col]
            p2 - point 2, [row, col]

        return:
            True if there are obstacles between two points
        '''
        ### YOUR CODE HERE ###
        
        x = np.linspace(p1[0], p2[0]).astype(int)
        #Coverx = int(np.linspace(p1[0], p2[0]))
        y = np.linspace(p1[1], p2[1]).astype(int)
        #Covery = int(np.linspace(p1[1], p2[1]))
        points = zip(x, y)
        
        isObstacle = False 

        for i in points:
            if self.map_array[i[0], i[1]] == 0:
                isObstacle = True
                
        return isObstacle 
        
    
    def dis(self, point1, point2):
        '''Calculate the euclidean distance between two points
        arguments:
            p1 - point 1, [row, col]
            p2 - point 2, [row, col]

        return:
            euclidean distance between two points
        '''
        ### YOUR CODE HERE ###

        dist = math.hypot((point1[0] - point2[0]), (point1[1] - point2[1]))
        
        return dist 


    def uniform_sample(self, n_pts):
        '''Use uniform sampling and store valid points
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points

        check collision and append valide points to self.samples
        as [(row1, col1), (row2, col2), (row3, col3) ...]
        '''
        # Initialize graph
        self.graph.clear()

        ### YOUR CODE HERE ###
        row = np.round(np.linspace(0, self.size_row - 1, num = int(math.sqrt(n_pts))), decimals = 0)  
        col = np.round(np.linspace(0, self.size_col - 1, num = int(math.sqrt(n_pts))), decimals = 0)
        
        #row = np.round(np.linspace(0, self.size_row - 1, int(math.sqrt(n_pts))), 0) 
        #col = np.round(np.linspace(0, self.size_col - 1, int(math.sqrt(n_pts))), 0)

        for row_new in row:
            for col_new in col:
                if self.map_array[int(row_new), int(col_new)] != 0:
                    self.samples.append((row_new, col_new))
                    
                    

    
    def random_sample(self, n_pts):
        '''Use random sampling and store valid points
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points

        check collision and append valide points to self.samples
        as [(row1, col1), (row2, col2), (row3, col3) ...]
        '''
        # Initialize graph
        self.graph.clear()

        ### YOUR CODE HERE ###

        points = []
        
        for _ in range(n_pts): 
            points.append((random.randint(self.size_row), random.randint(self.size_col)))
            
        for i in range(len(points)):
            x = points[i][0]
            y = points[i][1]
            if self.map_array[x][y]==1:
                self.samples.append((x, y))
            

    def gaussian_sample(self, n_pts):
        '''Use gaussian sampling and store valid points
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points

        check collision and append valide points to self.samples
        as [(row1, col1), (row2, col2), (row3, col3) ...]
        '''
        # Initialize graph
        self.graph.clear()

        ### YOUR CODE HERE ###
        self.samples.append((0, 0))

        points = []

        for _ in range(n_pts):
            if self.map_array[random.randint(self.size_row), random.randint(self.size_col)] == 0:                
                points.append((random.randint(self.size_row), random.randint(self.size_col)))
        
        
        for i in range(len(points)):
            isCollide = True
            while isCollide:   
                x = int(np.random.normal(points[i][0], 10))
                y = int(np.random.normal(points[i][1], 10))

                if x < self.size_row and y < self.size_col:
                    if self.map_array[(x, y)] == 1:
                        isCollide = False
                        self.samples.append((x,y))
                        
                    
                    

    def bridge_sample(self, n_pts):
        '''Use bridge sampling and store valid points
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points

        check collision and append valide points to self.samples
        as [(row1, col1), (row2, col2), (row3, col3) ...]
        '''
        # Initialize graph
        self.graph.clear()

        ### YOUR CODE HERE ###
        self.samples.append((0, 0))
        
        points = []
        
        for _ in range(n_pts):           
            if self.map_array[random.randint(self.size_row), random.randint(self.size_col)] == 0:                    
                points.append((random.randint(self.size_row), random.randint(self.size_col)))
        
        for i in range(len(points)):
            x, y = points[i][0], points[i][1]
            rand_x = int(np.random.normal(x, 13))
            rand_y = int(np.random.normal(y, 13))
        
            if rand_x < self.size_row and rand_y < self.size_col:
                if self.map_array[(rand_x, rand_y)] == 0:
                
                    point_x = int((x + rand_x) / 2)
                    point_y = int((y + rand_y) / 2)
                    if self.map_array[(point_x, point_y)] == 1:                                            
                        self.samples.append((point_x, point_y))
                         


    def draw_map(self):
        '''Visualization of the result
        '''
        # Create empty map
        fig, ax = plt.subplots()
        img = 255 * np.dstack((self.map_array, self.map_array, self.map_array))
        ax.imshow(img)

        # Draw graph
        # get node position (swap coordinates)
        node_pos = np.array(self.samples)[:, [1, 0]]
        pos = dict( zip( range( len(self.samples) ), node_pos) )
        pos['start'] = (self.samples[-2][1], self.samples[-2][0])
        pos['goal'] = (self.samples[-1][1], self.samples[-1][0])
        
        # draw constructed graph
        nx.draw(self.graph, pos, node_size=3, node_color='y', edge_color='y' ,ax=ax)

        # If found a path
        if self.path:
            # add temporary start and goal edge to the path
            final_path_edge = list(zip(self.path[:-1], self.path[1:]))
            nx.draw_networkx_nodes(self.graph, pos=pos, nodelist=self.path, node_size=8, node_color='b')
            nx.draw_networkx_edges(self.graph, pos=pos, edgelist=final_path_edge, width=2, edge_color='b')

        # draw start and goal
        nx.draw_networkx_nodes(self.graph, pos=pos, nodelist=['start'], node_size=12,  node_color='g')
        nx.draw_networkx_nodes(self.graph, pos=pos, nodelist=['goal'], node_size=12,  node_color='r')

        # show image
        plt.axis('on')
        ax.tick_params(left=True, bottom=True, labelleft=True, labelbottom=True)
        plt.show()


    def sample(self, n_pts=1000, sampling_method="uniform"):
        '''Construct a graph for PRM
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points
            sampling_method - name of the chosen sampling method

        Sample points, connect, and add nodes and edges to self.graph
        '''
        # Initialize before sampling
        self.samples = []
        self.graph.clear()
        self.path = []

        # Sample methods
        if sampling_method == "uniform":
            self.uniform_sample(n_pts)
            radius = 20
        elif sampling_method == "random":
            self.random_sample(n_pts)
            radius = 20
        elif sampling_method == "gaussian":
            self.gaussian_sample(n_pts)
            radius = 40
        elif sampling_method == "bridge":
            self.bridge_sample(n_pts)
            radius = 40 
        ### YOUR CODE HERE ###

        # Find the pairs of points that need to be connected
        # and compute their distance/weight.
        # Store them as
        # pairs = [(p_id0, p_id1, weight_01), (p_id0, p_id2, weight_02), 
        #          (p_id1, p_id2, weight_12) ...]
        pairs_vector = []    

        # Use sampled points and pairs of points to build a graph.
        # To add nodes to the graph, use
        # self.graph.add_nodes_from([p_id0, p_id1, p_id2 ...])
        # To add weighted edges to the graph, use
        # self.graph.add_weighted_edges_from([(p_id0, p_id1, weight_01), 
        #                                     (p_id0, p_id2, weight_02), 
        #                                     (p_id1, p_id2, weight_12) ...])
        # 'p_id' here is an integer, representing the order of 
        # current point in self.samples
        # For example, for self.samples = [(1, 2), (3, 4), (5, 6)],
        # p_id for (1, 2) is 0 and p_id for (3, 4) is 1.

        p_id = np.array(self.samples)
        kdtree = spatial.KDTree(p_id) 
        pairs = list(kdtree.query_pairs(radius))                             

        for i in range(len(pairs)):
            isCollision = self.check_collision(self.samples[pairs[i][0]], self.samples[pairs[i][1]])   
            
            if isCollision == False:
                dist = self.dis(self.samples[pairs[i][0]], self.samples[pairs[i][1]])                
                if dist != 0:
                    x = self.samples.index(self.samples[pairs[i][0]])
                    y = self.samples.index(self.samples[pairs[i][1]])
                    pairs_vector.append((x, y, dist))
    
            else:
                continue 

        self.graph.add_nodes_from([])
        self.graph.add_weighted_edges_from(pairs_vector)

        # Print constructed graph information
        n_nodes = self.graph.number_of_nodes()
        n_edges = self.graph.number_of_edges()
        print("The constructed graph has %d nodes and %d edges" %(n_nodes, n_edges))
        
        
        
    def search(self, start, goal):
        '''Search for a path in graph given start and goal location
        arguments:
            start - start point coordinate [row, col]
            goal - goal point coordinate [row, col]

        Temporary add start and goal node, edges of them and their nearest neighbors
        to graph for self.graph to search for a path.
        '''
        # Clear previous path
        self.path = []

        # Temporarily add start and goal to the graph
        self.samples.append(start)
        self.samples.append(goal)
        # start and goal id will be 'start' and 'goal' instead of some integer
        self.graph.add_nodes_from(['start', 'goal'])

        ### YOUR CODE HERE ###

        # Find the pairs of points that need to be connected
        # and compute their distance/weight.
        # You could store them as
        # start_pairs = [(start_id, p_id0, weight_s0), (start_id, p_id1, weight_s1), 
        #                (start_id, p_id2, weight_s2) ...]
        
        #Initializing an start and goal empty list 
        start_pairs = []
        goal_pairs = []  

        #Setting up a goal radius
        goal_radius = 60
        
        for i in range(len(self.samples)):
            
            start = self.samples[len(self.samples) - 2]  
            point = self.samples[i]
            
            if start != point:   
                if self.check_collision(start, point) == False:
                    dist = self.dis(start, point)
                    if dist != 0 and dist < goal_radius:                      
                        start_pairs.append(('start', self.samples.index(point), dist)) 
                else:
                    continue 
                                                       
        for i in range(len(self.samples)):
            
            goal = self.samples[len(self.samples) - 1]  
            point = self.samples[i]
            
            if goal != point: 
                
                if self.check_collision(goal,point) == False:
                    dist = self.dis(goal, point)
                    if dist != 0 and dist < goal_radius:
                        goal_pairs.append(('goal', self.samples.index(point), dist))

                else:
                    continue 

        # Add the edge to graph
        self.graph.add_weighted_edges_from(start_pairs)
        self.graph.add_weighted_edges_from(goal_pairs)
        
        # Seach using Dijkstra
        try:
            self.path = nx.algorithms.shortest_paths.weighted.dijkstra_path(self.graph, 'start', 'goal')
            path_length = nx.algorithms.shortest_paths.weighted.dijkstra_path_length(self.graph, 'start', 'goal')
            print("The path length is %.2f" %path_length)
        except nx.exception.NetworkXNoPath:
            print("No path found")
        
        # Draw result
        self.draw_map()

        # Remove start and goal node and their edges
        self.samples.pop(-1)
        self.samples.pop(-1)
        self.graph.remove_nodes_from(['start', 'goal'])
        self.graph.remove_edges_from(start_pairs)
        self.graph.remove_edges_from(goal_pairs)
        