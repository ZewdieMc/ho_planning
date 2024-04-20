import numpy as np
from scipy.spatial import KDTree

#* Class to implement the RRT algorithm
class Planner:
    def  __init__(self, state_validity_checker, max_iterations=10000, delta_q=2, p_goal=0.2, dominion=[-10, 10, -10, 10]):
        # define constructor ...
        self.state_validity_checker = state_validity_checker
        self.max_iterations = max_iterations
        self.delta_q = delta_q
        self.p_goal = p_goal
        self.dominion = dominion

        ##
        self.nodes = []
        self.parent = []
    
    def compute_path(self, q_start, q_goal):
        # Implement RRT algorithm.
        #* Use the state_validity_checker object to see if a position is valid or not.
        self.nodes = [q_start]
        self.parent = [0]

        for _ in range(self.max_iterations):
            q_rand = q_goal if np.random.rand() < self.p_goal else self.random_configuration()
            q_near, near_idx = self.nearest(q_rand)
            q_new = self.steer(q_near, q_rand)

            if self.state_validity_checker.is_valid(q_new) and self.state_validity_checker.check_path([q_near, q_new]):
                self.nodes.append(q_new)
                self.parent.append(near_idx)
                
                # avoid overshooting the goal #! needs to be checked
                if np.linalg.norm(q_new - q_goal) < self.delta_q and self.state_validity_checker.check_path([q_new, q_goal]):
                    self.nodes.append(q_goal)
                    self.parent.append(len(self.nodes) - 2)
                    raw_path = self.build_path(q_new)
                    smoothed_path = self.smooth_path(raw_path)
                    return smoothed_path
        return []
        
    
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
        idx = len(self.nodes) - 1

        while idx != 0:
            path.append(self.nodes[idx])
            idx = self.parent[idx]

        path.append(self.nodes[0])
        return path[::-1]