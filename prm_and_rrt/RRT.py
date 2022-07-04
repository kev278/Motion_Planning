# Standard Algorithm Implementation
# Sampling-based Algorithms RRT and RRT*

import matplotlib.pyplot as plt
import numpy as np
from random import randrange 
from numpy import random 
import math 


# Class for each tree node
class Node:
    def __init__(self, row, col):
        self.row = row        # coordinate
        self.col = col        # coordinate
        self.parent = None    # parent node
        self.cost = 0.0       # cost


# Class for RRT
class RRT:
    # Constructor
    def __init__(self, map_array, start, goal):
        self.map_array = map_array            # map array, 1->free, 0->obstacle
        self.size_row = map_array.shape[0]    # map size
        self.size_col = map_array.shape[1]    # map size

        self.start = Node(start[0], start[1]) # start node
        self.goal = Node(goal[0], goal[1])    # goal node
        self.vertices = []                    # list of nodes
        self.found = False                    # found flag
        

    def init_map(self):
        '''Intialize the map before each search
        '''
        self.found = False
        self.vertices = []
        self.vertices.append(self.start)

    
    def dis(self, node1, node2):
        '''Calculate the euclidean distance between two nodes
        arguments:
            node1 - node 1
            node2 - node 2

        return:
            euclidean distance between two nodes
        '''
        ### YOUR CODE HERE ###
        
        dist = math.hypot((node1.row - node2.row), (node1.col - node2.col))
        return dist 


    
    def check_collision(self, node1, node2):
        '''Check if the path between two nodes collide with obstacles
        arguments:
            node1 - node 1
            node2 - node 2

        return:
            True if the new node is valid to be connected
        '''
        ### YOUR CODE HERE ###
        #point = zip(int(np.linspace(node1.row, node2.row)), int(np.linspace(node1.col, node2.col)))
        point = zip(np.linspace(node1.row, node2.row).astype(int), np.linspace(node1.col, node2.col).astype(int))

        for i in point:
            if self.map_array[i[0]][i[1]] == 0:
                return True
        return False
    



    def get_new_point(self, goal_bias):
        '''Choose the goal or generate a random point
        arguments:
            goal_bias - the possibility of choosing the goal instead of a random point

        return:
            point - the new point
        '''
        ### YOUR CODE HERE ###
       
        goal = randrange(100)
        
        if goal <= goal_bias: 
            return self.goal
        else: 
            return Node(random.randint(self.size_row), random.randint(self.size_col))
    
    def get_nearest_node(self, point):
        '''Find the nearest node in self.vertices with respect to the new point
        arguments:
            point - the new point

        return:
            the nearest node
        '''
        ### YOUR CODE HERE ###
        length_vector = []
        
        for i in range(len(self.vertices)):
            dist = self.dis(point, self.vertices[i])
            length_vector.append(dist)
        
        return self.vertices[length_vector.index(min(length_vector))]
    
    
    def get_neighbors(self, new_node, neighbor_size):
        '''Get the neighbors that are within the neighbor distance from the node
        arguments:
            new_node - a new node
            neighbor_size - the neighbor distance

        return:
            neighbors - a list of neighbors that are within the neighbor distance 
        '''
        ### YOUR CODE HERE ###

        isNeighbor = []
        neighbor = []
        
        for i in range(len(self.vertices)):  
            
            dist = self.dis(new_node, self.vertices[i])
            
            if dist <= neighbor_size:                     
                isNeighbor.append(self.vertices[i])

        for i in range(len(isNeighbor)):                        
            isCollide = self.check_collision(isNeighbor[i], new_node)
            
            if isCollide == False:
                neighbor.append(isNeighbor[i])
            else:
                continue
            
        return neighbor        

    
    
    def rewire(self, new_node, neighbors):
        '''Rewire the new node and all its neighbors
        arguments:
            new_node - the new node
            neighbors - a list of neighbors that are within the neighbor distance from the node

        Rewire the new node if connecting to a new neighbor node will give least cost.
        Rewire all the other neighbor nodes.
        '''
        ### YOUR CODE HERE ###
        if neighbors == []:
            return -1

        dist_list = []
        points = []
        
        for i in range(len(neighbors)):    
            if self.check_collision(neighbors[i], new_node) == True:
                continue
            else:
                points.append(neighbors[i])
        
        for i in range(len(points)):                             
            dist = points[i].cost + self.dis(points[i], new_node)
            dist_list.append(dist)
        
        minIndex = dist_list.index(min(dist_list))
        new_node.parent = neighbors[minIndex]                     
        new_node.cost = neighbors[minIndex].cost + int(self.dis(neighbors[minIndex], new_node)) 

        for i in range(len(points)):                             
            prev_cost = points[i].cost                           
            new_cost = int(self.dis(new_node, points[i])) + new_node.cost

            if prev_cost > new_cost:
                neighbors[i].cost = new_cost
                neighbors[i].parent = new_node
                
            else:
                continue
    

    def draw_map(self):
        '''Visualization of the result
        '''
        # Create empty map
        fig, ax = plt.subplots(1)
        img = 255 * np.dstack((self.map_array, self.map_array, self.map_array))
        ax.imshow(img)

        # Draw Trees or Sample points
        for node in self.vertices[1:-1]:
            plt.plot(node.col, node.row, markersize=3, marker='o', color='y')
            plt.plot([node.col, node.parent.col], [node.row, node.parent.row], color='y')
        
        # Draw Final Path if found
        if self.found:
            cur = self.goal
            while cur.col != self.start.col and cur.row != self.start.row:
                plt.plot([cur.col, cur.parent.col], [cur.row, cur.parent.row], color='b')
                cur = cur.parent
                plt.plot(cur.col, cur.row, markersize=3, marker='o', color='b')

        # Draw start and goal
        plt.plot(self.start.col, self.start.row, markersize=5, marker='o', color='g')
        plt.plot(self.goal.col, self.goal.row, markersize=5, marker='o', color='r')

        # show image
        plt.show()


    def RRT(self, n_pts=1000):
        '''RRT main search function
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points

        In each step, extend a new node if possible, and check if reached the goal
        '''
        # Remove previous result
        self.init_map()

        ### YOUR CODE HERE ###

        # In each step,
        # get a new point, 
        # get its nearest node, 
        # extend the node and check collision to decide whether to add or drop,
        # if added, check if reach the neighbor region of the goal.

        goal_bias = 5                               
        step_size = 1
        
        for _ in range(n_pts):
            point = self.get_new_point(goal_bias)
            if self.map_array[point.row][point.col] == 1: 
                nearest_node = self.get_nearest_node(point)

                dist = self.dis(nearest_node, point)
                if dist > step_size:
                    dist = step_size
                
                col = nearest_node.col - point.col
                row = nearest_node.row - point.row
                theta = math.atan2(col, row)
                new_row = int(point.row + dist * math.cos(theta))
                new_col = int(point.col + dist * math.sin(theta))
                new_node = Node(new_row, new_col)
            

                collision = self.check_collision(nearest_node, new_node) 
                if collision == False:
                    self.vertices.append(new_node)  
                    new_node.parent = nearest_node  
                    new_node.cost = int(nearest_node.cost + self.dis(nearest_node, new_node))                     

                    if self.dis(new_node, self.goal) <= step_size:             
                        temp_node = new_node               
                        nearest_node = temp_node
                        new_node  = self.goal 
                        
                        new_node.cost = int(nearest_node.cost + self.dis(nearest_node, new_node))
                        new_node.parent = nearest_node  
                        
                        self.vertices.append(new_node)
                        self.found = True 

                        break   
                else: 
                    continue 
            else: 
                continue 


        # Output
        if self.found:
            steps = len(self.vertices) - 2
            length = self.goal.cost
            print("It took %d nodes to find the current path" %steps)
            print("The path length is %.2f" %length)
        else:
            print("No path found")
        
        # Draw result
        self.draw_map()


    def RRT_star(self, n_pts=1000, neighbor_size=20):
        '''RRT* search function
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points
            neighbor_size - the neighbor distance
        
        In each step, extend a new node if possible, and rewire the node and its neighbors
        '''
        # Remove previous result
        self.init_map()

        ### YOUR CODE HERE ###

        # In each step,
        # get a new point, 
        # get its nearest node, 
        # extend the node and check collision to decide whether to add or drop,
        # if added, rewire the node and its neighbors,
        # and check if reach the neighbor region of the goal if the path is not found.

        goal_bias = 5
        step_size = 1
        
        check = False
        
        for _ in range(n_pts):
            point = self.get_new_point(goal_bias)   
            
            if self.map_array[point.row][point.col] == 1:
                nearest_node  = self.get_nearest_node(point) 
                
                if check == False:                            
                    dist = self.dis(nearest_node, point)
                    if dist > step_size:
                        dist = step_size
                    
                    col = nearest_node.col - point.col
                    row = nearest_node.row - point.row
                    theta = math.atan2(col, row)
                    new_row = int(point.row + dist * math.cos(theta))
                    new_col = int(point.col + dist * math.sin(theta))
                    new_node = Node(new_row, new_col)

                    
                else:
                    new_node = self.goal
                
                if self.check_collision(new_node,nearest_node) == False:
                    self.rewire(new_node, self.get_neighbors(new_node, 20))  
                    dist = self.dis(new_node,self.goal)  

                    if dist <= step_size:
                        check = True     

                    self.vertices.append(new_node)

                    if dist == 0:
                        check = False
                        self.found = True
                    
                    else: 
                        continue  
            else: 
                continue 

        # Output
        if self.found:
            steps = len(self.vertices) - 2
            length = self.goal.cost
            print("It took %d nodes to find the current path" %steps)
            print("The path length is %.2f" %length)
        else:
            print("No path found")

        # Draw result
        self.draw_map()