# Class to implement the RRT* algorithm
import time

import numpy as np
from scipy.spatial import KDTree


class Planner:
    def __init__(self, state_validity_checker, max_iterations=10000, delta_q=2, p_goal=0.2, dominion=[-10, 10, -10, 10], search_radius=5, time_limit= 5):
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
        
        # Time span for the planner
        self.time_limit = time_limit

    def compute_path(self, q_start, q_goal):
        time_start = time.time()
        # Implement RRT* algorithm.
        self.nodes = [q_start]
        self.parent = [0]
        self.cost = [0]

        for _ in range(self.max_iterations):
            q_rand = q_goal if np.random.rand() < self.p_goal else self.random_configuration()
            q_near, near_idx = self.nearest(q_rand)
            q_new = self.steer(q_near, q_rand)

            if self.state_validity_checker.is_valid(q_new) and self.state_validity_checker.check_path([q_near, q_new]):
                neighbors = self.near(q_new)
                min_cost = self.cost[near_idx] + np.linalg.norm(q_new - q_near)
                min_idx = near_idx

                # Connect along the minimum cost path
                for i in neighbors:
                    new_cost = self.cost[i] + np.linalg.norm(q_new - self.nodes[i])
                    if self.state_validity_checker.check_path([self.nodes[i], q_new]) and new_cost < min_cost:
                        min_cost = new_cost 
                        min_idx = i

                self.nodes.append(q_new)
                self.parent.append(min_idx)
                self.cost.append(min_cost)

                # Rewire the tree
                for i in neighbors:
                    if self.state_validity_checker.check_path([q_new, self.nodes[i]]) and self.cost[-1] + np.linalg.norm(self.nodes[i] - q_new) < self.cost[i]:
                        self.parent[i] = len(self.nodes) - 1
                        self.cost[i] = self.cost[-1] + np.linalg.norm(self.nodes[i] - q_new)

                # Check if the goal can be reached
                # if np.linalg.norm(q_new - q_goal) < self.delta_q and self.state_validity_checker.check_path([q_new, q_goal]):
                #     self.path_to_goal_found = True
                #     self.nodes.append(q_goal)
                #     self.parent.append(len(self.nodes) - 2)
                #     self.cost.append(min_cost + np.linalg.norm(q_goal - q_new))
                    # raw_path = self.build_path(q_goal)
                    # smoothed_path = self.smooth_path(raw_path)
                    # return smoothed_path, self.parent, self.nodes
            if time.time() - time_start > self.time_limit:
                print("Time limit reached")
                # self.nodes.append(q_goal)
                raw_path = self.build_path(q_goal)
                smoothed_path = self.smooth_path(raw_path)
                return smoothed_path, self.parent, self.nodes
                # return [] #! No solution found within time limit, try with a higher time limit
        return []
    

    #* Find all nodes within a certain radius of q_new
    def near(self, q_new):
        radius = self.search_radius
        nodes_in_radius = []
        for i in range(len(self.nodes)):
            if np.linalg.norm(np.array(self.nodes[i]) - np.array(q_new)) < radius:
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
        return np.random.rand(2) * (self.dominion[1] - self.dominion[0]) + self.dominion[0]

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

        path.append(self.nodes[0])
        return path[::-1]
    