#!/usr/bin/python3

import rospy
from  nav_msgs.msg import Odometry, OccupancyGrid, GridCells
from geometry_msgs.msg import PoseWithCovarianceStamped, Point, Pose
from visualization_msgs.msg import Marker
import numpy as np
import math
from numpy import cos, sin
from utils.state_validity_checker import StateValidityChecker


class FrontierPlanner:
    def __init__(self, gridmap_topic, odom_topic):
        # Config Parameters 
        self.update_interval = 1 # update frontiers every x seconds
        
        # Class Module
        self.svc = StateValidityChecker()

        # Class Parameters
        self.last_map_time = rospy.Time.now()
        self.current_gridmap = None

        # PUBLISHER
        self.frontiers_pub = rospy.Publisher('~frontiers',GridCells,queue_size=1)
        self.viewpoints_pub = rospy.Publisher('~viewpoints',Marker,queue_size=1)

        # SUBSCRIBERS
        self.gridmap_sub = rospy.Subscriber(gridmap_topic,OccupancyGrid, self.gridmap_cb) 
        #self.odom_sub = rospy.Subscriber(odom_topic,Odometry, self.get_odom) 

        


    ##################################################################
    #### Callback functions
    ##################################################################
    def gridmap_cb(self, data):
        '''
        callback for gridmap
            - 
        '''
        if (data.header.stamp - self.last_map_time).to_sec() > self.update_interval:      
      
            self.last_map_time = data.header.stamp
            self.current_gridmap = data

            # Update State Validity Checker
            env = np.array(data.data).reshape(data.info.height, data.info.width).T
            origin = [data.info.origin.position.x, data.info.origin.position.y]
            self.svc.set(env, data.info.resolution, origin)

            
            # Prepare gridmap
            grid_map = self.prepare_grid_map(data)
            # Detect frontiers cells
            self.frontiers, frontiers_map = self.detect_frontiers(grid_map)
                    
            # Clustering
            self.clusters = self.group_frontiers(frontiers_map, self.frontiers)

            # Find viewpoint (list of pose -- need svc)
            self.viewpoints = self.find_viewpoint(self.clusters)
            print(self.viewpoints)
            # Find NBVP
            self.goal = self.find_nbvp(self.viewpoints)

            # Update visulization
            self.visualize()

    ##################################################################
    #### Alogorithm Functions
    ##################################################################
    def detect_frontiers(self,data):
        #Get shape of the data
        height, width = data.shape
        
        #Initialize frontiers list and map
        frontiers = []
        frontiers_map = np.zeros(data.shape)
        

        #Define direction vectors
        directions = [[1,0], [0,1], [-1,0], [0,-1]]

        #Get the free cells
        free_cells = np.where(data==255)#! not efficient...
        # Iterate over the free cells
        for i, j in zip(*free_cells):
            # Check the neighboring cells
            for dx, dy in directions:
                nx, ny = i + dx, j + dy
                # If the neighboring cell is within the grid and is an unknown cell
                if 0 <= nx < height and 0 <= ny < width and data[nx, ny] == 150:
                    # Add the coordinates of the free cell to the list
                    frontiers.append((i, j))
                    frontiers_map[i,j] = 1
                    # No need to check other directions
                    break

        # Return the list of free cells in contact with unknown cells
        return frontiers, frontiers_map
    
    def group_frontiers(self, map, frontiers):
        clusters = []
        visited_map = np.zeros(map.shape)

        for f in frontiers: # cell = [i, j]
            if not visited_map[tuple(f)]: 
                cluster = []
                stack = [f]

                while stack:
                    current = stack.pop()
                    visited_map[tuple(current)] = 1
                    if current not in cluster:
                        cluster.append(current)
                        neighbors = self.get_neighbor_cells(map, current,3)
                        for neighbor in neighbors:
                            if self.is_frontier(neighbor,map) and not self.is_visited(neighbor,visited_map):
                                stack.append(neighbor)

                clusters.append(cluster)

        final_clusters = []
        for c in clusters: # eleminate small clusters
            if len(c) > 10:
                final_clusters.append(c)
        return final_clusters


    
    def find_viewpoint(self, clusters):
        viewpoints = []
        for c in clusters:
            avg = np.mean(np.array(c), axis=0)
            viewpoints.append([avg[0],avg[1]])
        return viewpoints
    
    def find_nbvp(self, viewpoints):
        nbvp = viewpoints[0]
        return nbvp
    
    ##################################################################
    #### Utility functions
    ##################################################################
    def visualize(self):
        self.visualize_frontiers()
        self.visualize_viewpoints()
        

    def visualize_frontiers(self):
        # Create a GridCells message
        grid_msg = GridCells()
        grid_msg.header.frame_id = self.current_gridmap.header.frame_id  # Set the coordinate frame ID
        grid_msg.cell_width = self.current_gridmap.info.resolution  # Width of each cell
        grid_msg.cell_height = self.current_gridmap.info.resolution  # Height of each cell

        # Create some example cells (points)
        cells = []
        for f in self.frontiers:
            p = self.svc.__map_to_position__(f)
            point = Point()
            point.x = p[0]  # Example x-coordinate
            point.y = p[1]  # Example y-coordinate
            point.z = 0  # Ignored in 2D grids
            cells.append(point)

        grid_msg.cells = cells  # Assign the cells to the message
        self.frontiers_pub.publish(grid_msg)

    def visualize_viewpoints(self):
        
        marker = Marker()
        marker.header.frame_id = self.current_gridmap.header.frame_id  # Marker's reference frame
        marker.header.stamp = rospy.Time.now()
        marker.ns = "viewpoint"  # Namespace to avoid conflicts
        marker.id = 0  # Marker ID
        marker.type = Marker.SPHERE_LIST  # Type of marker (SPHERE_LIST)
        marker.action = Marker.ADD  # Action to take (ADD, DELETE, etc.)
        marker.scale.x = 0.3  # Scale (diameter along x-axis)
        marker.scale.y = 0.3  # Scale (diameter along y-axis)
        marker.scale.z = 0.3  # Scale (diameter along z-axis)
        marker.color.a = 1.0  # Alpha (transparency)
        marker.color.r = 1.0  # Red component
        marker.color.g = 0.0  # Green component
        marker.color.b = 0.0  # Blue component
        marker.lifetime = rospy.Duration()  # Marker lifetime (0 means forever)
        marker.pose = Pose()

        # Create some example sphere points
        sphere_points = []
        for i, vp in enumerate(self.viewpoints):
            p = self.svc.__map_to_position__(vp)
            point = Point()
            point.x = p[0]  # Example x-coordinate
            point.y = p[1]  # Example y-coordinate
            point.z = 0.5  # Example z-coordinate
            sphere_points.append(point)

        marker.points = sphere_points
        self.viewpoints_pub.publish(marker)

    

    ##################################################################
    #### Utility functions
    ##################################################################
    def prepare_grid_map(self, data):
        '''
        turn grid_map msg to np.array 2ith value 0, 0.5 and 1.0
        '''
        grid_map =  np.array(data.data).reshape(data.info.height, data.info.width).T
        grid_map_copy= np.where(grid_map==-1,150,grid_map) # replace -1 with 150
        grid_map_copy= np.where(grid_map==100,0,grid_map_copy) # replace 100 with 0
        grid_map_copy= np.where(grid_map==0,255,grid_map_copy) # replace 0 with 255
        return grid_map_copy
    
    def is_frontier(self, cell, map):
        return bool(map[tuple(cell)] == 1.0) 

    def is_visited(self, cell, map):
        return bool(map[tuple(cell)] == 1.0) 


    def get_neighbor_cells(self, map, cell, radius=1):
        neighbors = []
        x, y = cell[0], cell[1]
        for delta_x, delta_y in [
            (-1, -1),
            (0, -1),
            (1, -1),
            (1, 0),
            (1, 1),
            (0, 1),
            (-1, 1),
            (-1, 0),
        ]:
            for i in range(radius):
            # get neighbor cell
                nx, ny = x + delta_x*(i+1), y + delta_y*(i+1)
                if 0 <= nx < map.shape[0] and 0 <= ny < map.shape[1]:
                    neighbors.append([nx, ny]) 
        return neighbors


if __name__ == '__main__':

    rospy.init_node('frontier_planner')
    robot = FrontierPlanner("/projected_map", "/odom")
    rospy.loginfo("Frontier Planner node started")

    rospy.spin()