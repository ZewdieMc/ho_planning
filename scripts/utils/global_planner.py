from ast import List
import time
import numpy as np
from scipy.spatial import KDTree
from collections import OrderedDict
# from utils_lib.rrt import RRT

def wrap_angle(angle):
    return (angle + ( 2.0 * np.pi * np.floor( ( np.pi - angle ) / ( 2.0 * np.pi ) ) ) )

class StateValidityChecker:
    """ Checks if a position or a path is valid given an occupancy map."""

    # Constructor
    def __init__(self, distance=0.4, is_unknown_valid=True):
        # map: 2D array of integers which categorizes world occupancy
        self.map = None 
        # map sampling resolution (size of a cell))                            
        self.resolution = None
        # world position of cell (0, 0) in self.map                      
        self.origin = None
        # set method has been called                          
        self.there_is_map = False
        # radius arround the robot used to check occupancy of a given position                 
        self.distance = distance                    
        # if True, unknown space is considered valid
        self.is_unknown_valid = is_unknown_valid    



    
    # Set occupancy map, its resolution and origin. 
    def set(self, data, resolution, origin):
        self.map = data
        self.resolution = resolution
        self.origin = np.array(origin)
        self.there_is_map = True
    
    # Given a pose, returs true if the pose is not in collision and false othewise.
    def is_valid(self, pose): 

        # ?convert world robot position to map coordinates using method __position_to_map__
        # ?check occupancy of the vicinity of a robot position (indicated by self.distance atribute). 
        #* Return True if free, False if occupied and self.is_unknown_valid if unknown. 
        #* If checked position is outside the map bounds consider it as unknown.
        
        cell = self.__position_to_map__(pose)

        # check if the cell is outside the map #* __position_to_map__ returns None if the cell is outside the map
        if cell is None:
            return self.is_unknown_valid 
            
        distance_in_cells = int(self.distance / self.resolution)
        # distance in cells
        for i in range(-distance_in_cells, distance_in_cells + 1): 
            for j in range(-distance_in_cells, distance_in_cells + 1):
                # compute the vicinity cell to check
                check_cell = cell + np.array([i, j])
                #* if the check_cell is outside the map, return is_unknown_valid
                if np.any(check_cell < 0) or np.any(check_cell >= self.map.shape):
                    if not self.is_unknown_valid:
                        return False
                    # return self.is_unknown_valid
                #* if the check_cell is unknown, return is_unknown_valid
                elif self.map[check_cell[0], check_cell[1]] == -1:
                    if not self.is_unknown_valid:
                        return False
                
                #* if the check_cell is occupied, return False
                elif self.map[check_cell[0], check_cell[1]] == 100:
                    return False
                
        return True
    

    # Given a path, returs true if the path is not in collision and false othewise.
    def check_path(self, path):

        # TODO: Discretize the positions between 2 waypoints with an step_size = 2*self.distance
        # TODO: for each point check if `is_valid``. If only one element is not valid return False, otherwise True. 

        step_size = self.distance*2
        for i in range(len(path) - 1):
            start = path[i]
            end = path[i + 1]

            # Calculate distance and direction of the segment
            dist = np.linalg.norm(end - start)
            if dist == 0:
                continue
            direction = (end - start) / dist

            if dist < step_size:
                if not self.is_valid(start) or not self.is_valid(end):
                    return False
                continue

            # Discretize the segment and check each configuration
            num_steps = int(np.ceil(dist / step_size))
            for j in range(num_steps):
                c = start + j * step_size * direction
                if not self.is_valid(c):
                    return False
        return True

    # Transform position with respect the map origin to cell coordinates
    def __position_to_map__(self, p):

        # TODO: convert world position to map coordinates. 
        cell = np.floor((p - self.origin) / self.resolution).astype(int)
        if np.any(cell < 0) or np.any(cell >= self.map.shape):
            return None
        return cell
    
    # Transform cell coordinates to world position
    def __map_to_position__(self, cell):
        return self.origin + cell * self.resolution + self.resolution / 2
    
# Class to implement the RRT* algorithm
class Planner:
    def __init__(self, state_validity_checker, max_iterations=3000, delta_q=0.8, p_goal=0.5, dominion=[-10, 10, -10, 10], search_radius=2, time_limit= 7):
        # Define constructor ...
        self.state_validity_checker = state_validity_checker
        self.max_iterations = max_iterations
        self.delta_q = delta_q
        self.p_goal = p_goal
        self.dominion = dominion
        self.search_radius = search_radius
        #goal checker
        self.path_to_goal_found = False
        # Initialize nodes, parent, and cost lists
        self.nodes = []
        self.parent = []
        self.cost = []

        # iterations
        self.iterations = 0

        
        # Time span for the planner
        self.time_limit = time_limit

    def compute_path(self, q_start, q_goal):
        time_start = time.time()
        # Implement RRT* algorithm.
        goal_found_counter = 0
        node_found_counter = 0

        self.nodes = [q_start]
        self.nodes_dict = {tuple(q_start): 0}
        self.parent = [0]
        self.cost = [0]

        for _ in range(self.max_iterations):
            self.iterations += 1
            q_rand = q_goal if np.random.rand() < self.p_goal else self.random_configuration()
            
            q_near, near_idx = self.nearest(q_rand)
            q_new = self.steer(q_near, q_rand)
            
            if self.state_validity_checker.is_valid(q_new) and self.state_validity_checker.check_path([q_near, q_new]):
                neighbors = self.near(q_new, q_goal)
                min_cost = self.cost[near_idx] + np.linalg.norm(q_new - q_near)
                min_idx = near_idx

                # Connect along the minimum cost path
                for i in neighbors:
                    new_cost = self.cost[i] + np.linalg.norm(q_new - self.nodes[i])
                    if self.state_validity_checker.check_path([self.nodes[i], q_new]) and new_cost < min_cost:
                        min_cost = new_cost 
                        min_idx = i
                # Check if the goal is reached
                # if not self.path_to_goal_found:
                index = self.find_goal_index(q_new)
                
                # Goal already exists in the tree, check if new cost is less than the existing cost
                if index is not None:# and np.allclose(q_new, q_goal, atol=0.001):
                    node_found_counter += 1
                    if min_cost < self.cost[index]:
                        self.parent[index] = min_idx
                        self.cost[index] = min_cost
                        print("cost updated")
                    if np.allclose(q_new, q_goal, atol=0.001):
                        goal_found_counter += 1
                    
                else:         
                    self.nodes.append(q_new)
                    self.parent.append(min_idx)
                    self.cost.append(min_cost)
                    self.nodes_dict[tuple(q_new)] = min_cost

                # Rewire the tree
                for i in neighbors:
                    if self.state_validity_checker.check_path([q_new, self.nodes[i]]) and self.cost[-1] + np.linalg.norm(self.nodes[i] - q_new) < self.cost[i]:
                        self.parent[i] = len(self.nodes) - 1
                        self.cost[i] = self.cost[-1] + np.linalg.norm(self.nodes[i] - q_new)
                        
            if time.time() - time_start > self.time_limit:
                raw_path = self.build_path(q_goal)
                smoothed_path = self.smooth_path(raw_path)
                return smoothed_path, self.parent, self.nodes
        return [], [], []

    def find_goal_index(self, q_goal):
        for i, node in enumerate(self.nodes):
            if np.allclose(node, q_goal, atol=0.001):
                self.path_to_goal_found = True
                return i
        return None


    def count_duplicates(self):
        duplicates = 0
        for i in range(len(self.nodes)):
            for j in range(i + 1, len(self.nodes)):
                if np.allclose(self.nodes[i], self.nodes[j], atol=0.001):
                    duplicates += 1
        return duplicates
    
    #* Find all nodes within a certain radius of q_new
    def near(self, q_new, q_goal):
        radius = self.search_radius
        nodes_in_radius = []
        distance_to_goal = np.linalg.norm(np.array(q_new) - np.array(q_goal))
        for i in range(len(self.nodes)):
            if np.linalg.norm(np.array(self.nodes[i]) - np.array(q_new)) < radius and distance_to_goal > 0.001:
                nodes_in_radius.append(i)
        return nodes_in_radius
    

    def smooth_path(self, input_path):
        goal = len(input_path) - 1
        path = list(input_path)

        while goal > 0:
            start = 0
            while start < goal - 1 and not self.state_validity_checker.check_path([path[start],path[goal]]):
                start += 1

            if start < goal - 1:
                # Shorter path found
                path = path[: start + 1] + path[goal:]
                goal = start
            else:
                goal -= 1

        return path
    
    #* Generate a random configuration
    def random_configuration(self):
       return [np.random.uniform(self.dominion[0], self.dominion[1]),np.random.uniform(self.dominion[2], self.dominion[3]) ]


    #* Find the nearest node in the tree to q_rand
    def nearest(self, q_rand):
        tree = KDTree(self.nodes)
        _, idx = tree.query(q_rand)
        return self.nodes[idx], idx
    
    #* Steer from from_node towards to_node
    def steer(self, from_node, to_node):
        direction = to_node - from_node
        distance = np.linalg.norm(direction)
        if distance > self.delta_q:
            direction = direction / distance * self.delta_q
        return from_node + direction
    
    #* Build path from start to last_node
    def build_path(self, last_node):
        path = []
        idx = None

        # Find the index of last_node in self.nodes
        for i, node in enumerate(self.nodes):
            if np.array_equal(node, last_node):
                idx = i
                break

        # If last_node is not in self.nodes, idx will be None
        if idx is None:
            return []

        while idx != 0:
            path.append(self.nodes[idx])
            idx = self.parent[idx]

        # path.append(self.nodes[0])
        return path[::-1]
    
# Planner: This function has to plan a path from start_p to goal_p. To check if a position is valid the 
# StateValidityChecker class has to be used. The planning dominion must be specified as well as the maximum planning time.
# The planner returns a path that is a list of poses ([x, y]).
def compute_path(start_p, goal_p, state_validity_checker, bounds, max_time=1.0):

    # TODO: Plan a path from start_p to goal_p inside bounds using the RRT and the StateValidityChecker Objects previously defined.
    rrt = Planner(state_validity_checker, dominion=bounds)
    path, parents, nodes = rrt.compute_path(start_p, goal_p)
    # TODO: if solved, return a list with the [x, y] points in the solution path.
    if path is not None and len(path) > 0:
        return path, parents, nodes
    
    return [], [], []
    
    # while path is None or len(path) == 0:
    #     path, parents, nodes = rrt.compute_path(start_p, goal_p)
        
    # return path, parents, nodes
# Controller: Given the current position and the goal position, this function computes the desired 
# lineal velocity and angular velocity to be applied in order to reah the goal.
def move_to_point(current, goal, Kv=0.5, Kw=0.5):
    d = ((goal[0] - current[0])**2 + (goal[1] - current[1])**2)**0.5
    psi_d = np.arctan2(goal[1] - current[1], goal[0] - current[0])
    psi = wrap_angle(psi_d - current[2])
    v = 0.0 if abs(psi) > 0.05 else Kv * d
    w = Kw * psi
    return v, w