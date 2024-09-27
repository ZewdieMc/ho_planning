from scipy.signal import convolve2d
from sklearn.cluster import DBSCAN

import numpy as np

#! Find the frontiers using convolution
def find_frontiers_conv(data):
    data_copy = data.copy() # copy the data
    data_copy= np.where(data==-1,150,data) # replace -1 with 150
    data_copy= np.where(data==100,0,data_copy) # replace 100 with 0
    data_copy= np.where(data==0,255,data_copy) # replace 0 with 255

    # Define the convolution kernel
    kernel = np.array([[0, 1, 0], [1, 0, 1], [0, 1, 0]])

    # Convolve the data with the kernel
    conv = convolve2d(data_copy == 150, kernel, mode='same')
    conv2 = convolve2d(data_copy == 255, kernel, mode='same')

    # Find the free cells that have an unknown neighbor and a free cell neighbor
    frontiers = np.where((data_copy == 255) & ((conv > 0) & (conv2 > 0)))
    # Return the list of free cells in contact with unknown cells or free cells
    return list(zip(*frontiers))


#! Cluster the frontiers using DBSCAN
def cluster_frontiers(frontiers):
    # Convert frontiers to a 2D NumPy array
    X = np.array(frontiers)

    # Run DBSCAN on the frontiers
    clustering = DBSCAN(eps=3, min_samples=5).fit(X)

    # Calculate the centroid of each cluster
    centroids = []
    frontiers_without_noise = []
    clusters = []
    for cluster_id in np.unique(clustering.labels_):
        if cluster_id != -1:  # Ignore noise points
            points_in_cluster = X[clustering.labels_ == cluster_id]
            clusters.append(points_in_cluster)
            centroid = compute_median(points_in_cluster)#np.median(points_in_cluster, axis=0)
            centroids.append(centroid)
            frontiers_without_noise.extend(points_in_cluster.tolist())

    return clusters, centroids, frontiers_without_noise

#! Naive method check before using it.
def compute_median(cluster_points):
    # Compute the median of the cluster points
    sorted_points = sorted(cluster_points, key=lambda x: x[0])
    mid = len(sorted_points) // 2
    if len(sorted_points) % 2 == 0:
        return sorted_points[mid - 1] 
    else:
        return sorted_points[mid]
    
# Dynamic window approach algorithm to select best v and w
def dwa(self, goal, v_max, w_max, dv_resolution, dw_resolution, dt, t_total): 

          grid_map = np.array(self.svc.map)

          weight_range = [0, 1]

          # Initial state
          self.current_pose
          xi, yi, theta_i = self.current_pose[0], self.current_pose[1], self.current_pose[2]
          pose = [xi, yi, theta_i]
          self.dwa_heading(pose)

          vi = 0

          goal = goal # m
          goal_theta = self.svc.compute_angle((xi, yi), goal) # rad
          goal_theta = self.correct_angle(goal_theta)

          goal_heading = [xi, yi, goal_theta]
          self.dwa_goal_heading(goal_heading)

          thresh = np.pi/6    
          angle_diff = abs(self.correct_angle(theta_i) - goal_theta)

          no_of_branches = len(np.arange(-w_max, w_max, dw_resolution))
          collision_list = [False] * no_of_branches

          # Empty lists
          trajectory = []

          speed_dict = {}
          goal_direction_dict = {}
          collision_dict = {}

          xy_to_vw = {}
          xy_list = []

          for v in np.arange(0, v_max, dv_resolution):
            for j, w in enumerate(np.arange(-w_max, w_max, dw_resolution)):
                x, y, theta = xi, yi, theta_i

                # simulate motion for the given command
                motion = []
                for _ in np.arange(0, t_total, dt):
                    x, y, theta = self.simulate_motion(v, w, dt, x, y, theta)
                    x = round(x, 2)
                    y = round(y, 2)

                    motion.append((x, y))
                    p = self.svc.__position_to_map__([y, x])
                    
                    if p == [] or grid_map[p[1], p[0]] != 0:
                        collision_list[j-1] = True
                
                trajectory.append(motion)

                #if x != xi and y!= yi:
                xy_to_vw[(x, y)] = (v, w)
                xy_list.append([x, y])

                speed_cost = v - vi
                speed_dict[(x, y)] = speed_cost

                theta = self.correct_angle(theta)

                goal_direction_cost =  abs(goal_theta - theta)
                goal_direction_dict[(x, y)] = goal_direction_cost
            
          for index, line in enumerate(trajectory):
            k = index
            while k > no_of_branches:
                k -= no_of_branches
            
            if not collision_list[k-1]:
                x, y = zip(*line)
                #if x[-1] != xi and y[-1] != yi:
                collision_cost = weight_range[1]
                collision_dict[(x[-1], y[-1])] = collision_cost
          
          self.dwa_tree(xy_list)

          # Compute weights
          new_speed_dict = self.svc.scale(speed_dict, weight_range)
          new_goal_direction_dict = self.svc.scale(goal_direction_dict, [weight_range[1], weight_range[0]])
          
          # Find best command
          result_dict = self.svc.add_dicts(new_speed_dict, new_goal_direction_dict, collision_dict)
          key_with_max_value = max(result_dict, key=result_dict.get)
          
          best_v, best_w = xy_to_vw[key_with_max_value]
          self.best_v_w_marker(key_with_max_value)

          # If goal is out of the window, switch to normal controller
          if angle_diff > thresh:
              best_v, best_w = ...#!move_to_point(self.current_pose, self.path[0])
              best_v = round(best_v, 2)
              best_w = round(best_w, 2)
        
          print("The best (v, w) command is", [best_v, best_w])
          #best_v, best_w = 0, 0

          return best_v, best_w
