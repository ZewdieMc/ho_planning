#!/usr/bin/python3

import numpy as np
import rospy
import tf

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Pose
from std_msgs.msg import ColorRGBA 
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from utils.global_planner import StateValidityChecker, move_to_point, compute_path
import actionlib
from ho_planning.msg import FollowPathAction, FollowPathGoal, FollowPathResult
from ho_planning.srv import GetGlobalPlan, GetGlobalPlanResponse
class OnlinePlanner:

    # OnlinePlanner Constructor
    def __init__(self, gridmap_topic, odom_topic, cmd_vel_topic, bounds, distance_threshold):

        # ATTRIBUTES
        # List of points which define the plan. None if there is no plan
        self.path = []
        self.copy_path = []
        # State Validity Checker object                                                 
        self.svc = StateValidityChecker(distance_threshold)
        # Current robot SE2 pose [x, y, yaw], None if unknown            
        self.current_pose = None
        # Goal where the robot has to move, None if it is not set                                                                   
        self.goal = None
        # Last time a map was received (to avoid map update too often)                                                
        self.last_map_time = rospy.Time.now()
        # Dominion [min_x_y, max_x_y] in which the path planner will sample configurations                           
        self.bounds = bounds                                        

        # # CONTROLLER PARAMETERS
        # # Proportional linear velocity controller gain
        # self.Kv = 2.5
        # # Proportional angular velocity controller gain                   
        # self.Kw = 5
        # # Maximum linear velocity control action                   
        # self.v_max = 0.15
        # # Maximum angular velocity control action               
        # self.w_max = 0.9           

        # #Frontiers
        # self.frontiers = []    
        # self.centroids = []
        self.current_gridmap = None
        # # tree nodes and edges
        self.nodes = []
        self.parent = []
        # # ACTION CLIENT
        # self.client = actionlib.SimpleActionClient('follow_path', FollowPathAction)
        # self.client.wait_for_server()

        # # PUBLISHERS
        # #? Publisher for sending velocity commands to the robot
        # self.path_pub = rospy.Publisher("/global_path",Path, queue_size=10)
        # #? Publisher for visualizing the path to with rviz
        # self.marker_pub = rospy.Publisher('~path_marker', Marker, queue_size=1)
        self.node_pub = rospy.Publisher('~rrt_nodes', Marker, queue_size=1)
        self.edge_pub = rospy.Publisher('~rrt_edges', Marker, queue_size=1)
        self.bounds_pub = rospy.Publisher('~rrt_bounds', Marker, queue_size=1)



        # #Frontier publisher
        # self.frontier_pub = rospy.Publisher('~frontiers', MarkerArray, queue_size=1)
        # self.waypoints_pub = rospy.Publisher('/waypoints',PoseStamped,queue_size=10)

        # SUBSCRIBERS
        #?subscriber to gridmap_topic from Octomap Server  
        self.gridmap_sub = rospy.Subscriber(gridmap_topic, OccupancyGrid, self.get_gridmap)
        self.global_plan_srv = rospy.Service('get_global_plan', GetGlobalPlan, self.handle_get_global_plan)

        # #?subscriber to odom_topic  
        self.odom_sub = rospy.Subscriber(odom_topic, Odometry, self.get_odom)
        # ?subscriber to /move_base_simple/goal published by rviz
        # self.move_goal_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.get_goal)    
        
        # TIMERS
        # Timer for velocity controller
        # rospy.Timer(rospy.Duration(0.1), self.dwa_controller)
        rospy.Timer(rospy.Duration(1), self.visualize)


    def handle_get_global_plan(self, req):
        goal = np.array([req.goal_x, req.goal_y])
        resp = GetGlobalPlanResponse()

        if not self.svc.is_valid(goal):
            rospy.logerr("Goal is not valid, unable to plan a path")
            resp.success = False
            return resp


        rospy.loginfo("Computing new path")
        self.path, parents, nodes = compute_path(self.current_pose[0:2],goal, self.svc, self.bounds, 1.0)
        self.copy_path = self.path.copy()
        if len(self.path) > 0:
            self.nodes = nodes
            self.parent = parents
        else:
            resp.success = False
            rospy.logerr("No path found")
            return resp
        
        resp.success = True
        for i in range(len(self.path)):
            resp.path_x.append(self.path[i][0])
            resp.path_y.append(self.path[i][1])
        
        rospy.loginfo("Path found")
        return resp

    # Odometry callback: Gets current robot pose and stores it into self.current_pose
    def get_odom(self, odom):
        _, _, yaw = tf.transformations.euler_from_quaternion([odom.pose.pose.orientation.x, 
                                                              odom.pose.pose.orientation.y,
                                                              odom.pose.pose.orientation.z,
                                                              odom.pose.pose.orientation.w])

        #?Store current position (x, y, yaw) as a np.array in self.current_pose var.
        self.current_pose = np.array([odom.pose.pose.position.x, odom.pose.pose.position.y, yaw])
    
    #* Goal callback: Get new goal from /move_base_simple/goal topic published by rviz 
    # and computes a plan to it using self.plan() method
    def get_goal(self, goal):
        rospy.loginfo("goal: %s", goal)
        if self.svc.there_is_map:
            #?Store goal (x,y) as a numpy aray in self.goal var and print it 
            self.goal = np.array([goal.pose.position.x, goal.pose.position.y])
            self.path = []  # To prevent the robot from moving while planning
            print("New goal received: ", self.goal)
            if self.svc.is_valid(self.goal):
                #* Plan a new path to self.goal
                self.path = []
                self.plan()
                self.send_action()
            else:
                print("Goal is not valid, unable to plan a path")
        
    # Map callback: Gets the latest occupancy map published by Octomap server and update 
    # the state validity checker
    def get_gridmap(self, gridmap):
      
        # To avoid map update too often (change value '1' if necessary)
        if (gridmap.header.stamp - self.last_map_time).to_sec() > 1:            
            self.last_map_time = gridmap.header.stamp

            self.current_gridmap = gridmap
            # Update State Validity Checker
            env = np.array(gridmap.data).reshape(gridmap.info.height, gridmap.info.width).T
            origin = [gridmap.info.origin.position.x, gridmap.info.origin.position.y]
            self.svc.set(env, gridmap.info.resolution, origin)
            #! self.frontiers = exploration.find_frontiers_conv(env)
            map_height = self.svc.map.shape[0] * self.svc.resolution
            map_width = self.svc.map.shape[1] * self.svc.resolution

            self.bounds = [origin[0] - 0.5 , origin[0] + map_height + 0.5  , origin[1] - 0.5 , origin[1] + map_width + 0.5 ]
            

    def send_action(self):
        goal = FollowPathGoal()
        path_msg = self.discretize_path([self.current_pose[0:2]] + self.path)
        goal.path = path_msg
        self.path_pub.publish(path_msg)
        self.client.send_goal(goal)


    # Solve plan from current position to self.goal. 
    def plan(self):
        # Invalidate previous plan if available
        self.path = []
        if not self.svc.is_valid(self.goal):
            self.path = []
            rospy.logerr("Goal is not valid, unable to plan a path")
            return

        # Check if robot is collided with an obstacle
        if self.robot_collided():
            rospy.logerr("Robot is collided with an obstacle")
            self.back_off()

        rospy.loginfo("Computing new path")
        # TODO: plan a path from self.current_pose to self.goal
        self.path, parents, nodes = compute_path(self.current_pose[0:2], self.goal, self.svc, self.bounds, 1.0)
        self.copy_path = self.path.copy()
        if len(self.path) > 0:
            self.nodes = nodes
            self.parent = parents
        else:
            rospy.logerr("No path found")
            self.plan()
        self.publish_nodes()
        self.publish_edges()        
        rospy.loginfo("Path found")
        print("Path: ", self.path)
        # Publish plan marker to visualize in rviz
        # self.publish_path()
        # del self.path[0]                 
        
        # path_msg = self.discretize_segment(self.current_pose, self.path[0])
        # self.path_pub.publish(path_msg)
        # remove initial waypoint in the path (current pose is already reached)
        # del self.path[0]                 
        

    # This method is called every 0.1s. It computes the velocity comands in order to reach the 
    # next waypoint in the path. It also sends zero velocity commands if there is no active path.

    def controller(self, event):
        v = 0
        w = 0
        if len(self.path) > 0:
            # TODO: If current waypoint is reached with some tolerance move to the next waypoint.
            if np.linalg.norm(self.current_pose[0:2] - self.path[0]) < 0.01:
                # remove reached waypoint
                print("Waypoint deleted!", self.path[0])
                del self.path[0]

                # If it was the last waypoint in the path show a message indicating it
                if len(self.path) == 0:
                    print("Goal reached!") 

            else:
                # move to the next waypoint in the path
                v, w = move_to_point(self.current_pose, self.path[0], self.Kv, self.Kw)                
        # Publish velocity commands
        self.__send_commnd__(v, w)

    def dwa_controller(self, event):
        v = 0
        w = 0
        if len(self.path) > 0:
            # TODO: If current waypoint is reached with some tolerance move to the next waypoint.
            if np.linalg.norm(self.current_pose[0:2] - self.path[0]) < 0.16:
                # remove reached waypoint
                print("Waypoint deleted!", self.path[0])
                del self.path[0]
                

                # If it was the last waypoint in the path show a message indicating it
                if len(self.path) == 0:
                    print("Goal reached!") 
                else:
                    ...
                    # path_msg = self.discretize_segment(self.current_pose, self.path[0])
                    # self.path_pub.publish(path_msg)

            else:
                # move to the next waypoint in the path
                # print("velocity command to: ", self.path[0])
                # wp = PoseStamped()
                # wp.header.frame_id = self.current_gridmap.header.frame_id
                # wp.pose.position.x = self.path[0][0]
                # wp.pose.position.y = self.path[0][1]
                # self.waypoints_pub.publish(wp)    
                # path_msg = self.discretize_segment(self.current_pose, self.path[0])
                # self.path_pub.publish(path_msg)
                ...
        else:
            ...
            # rospy.loginfo("No path -- controller doing nothing")
    

    # PUBLISHER HELPERS
    # Transform linear and angular velocity (v, w) into a Twist message and publish it
    def __send_commnd__(self, v, w):
        cmd = Twist()
        cmd.linear.x = np.clip(v, -self.v_max, self.v_max)
        cmd.linear.y = 0
        cmd.linear.z = 0
        cmd.angular.x = 0
        cmd.angular.y = 0
        cmd.angular.z = np.clip(w, -self.w_max, self.w_max)
        self.cmd_pub.publish(cmd)


    def convert_frontiers_to_world(self, frontiers):
        world_frontiers = []
        for frontier in frontiers:
            world_frontiers.append([frontier[0] * self.svc.resolution + self.svc.origin[0], frontier[1] * self.svc.resolution + self.svc.origin[1]])
        return world_frontiers

    # Publish a path as a series of line markers
    def publish_path(self):
        if len(self.copy_path) > 1:
            print("Publish path!")
            m = Marker()
            m.header.frame_id = self.current_gridmap.header.frame_id
            m.header.stamp = rospy.Time.now()
            m.id = 0
            m.type = Marker.LINE_STRIP
            m.ns = 'path'
            m.action = Marker.DELETE
            m.lifetime = rospy.Duration(0)
            self.marker_pub.publish(m)

            m.action = Marker.ADD
            m.scale.x = 0.1
            m.scale.y = 0.0
            m.scale.z = 0.0
            
            m.pose.orientation.x = 0
            m.pose.orientation.y = 0
            m.pose.orientation.z = 0
            m.pose.orientation.w = 1
            
            color_red = ColorRGBA()
            color_red.r = 1
            color_red.g = 0
            color_red.b = 1
            color_red.a = 1
            color_blue = ColorRGBA()
            color_blue.r = 0
            color_blue.g = 0
            color_blue.b = 1
            color_blue.a = 1

            p = Point()
            p.x = self.current_pose[0]
            p.y = self.current_pose[1]
            p.z = 0.0
            m.points.append(p)
            m.colors.append(color_blue)
            
            for n in self.copy_path:
                p = Point()
                p.x = n[0]
                p.y = n[1]
                p.z = 0.0
                m.points.append(p)
                m.colors.append(color_red)
            
            self.marker_pub.publish(m)
        else:
            print("No path to publish")


    def back_off(self):
        #! Move the robot back a little bit
        backoff_t1 = rospy.Time.now()
        cmd = Twist()
        cmd.linear.x = np.clip(-0.1, -self.v_max, self.v_max)
        cmd.linear.y = 0
        cmd.linear.z = 0
        cmd.angular.x = 0
        cmd.angular.y = 0
        cmd.angular.z = 0
        rospy.logwarn("Moving the robot back")
        while (rospy.Time.now() - backoff_t1) < rospy.Duration(3):
            self.cmd_pub.publish(cmd) # move the robot back
        cmd.linear.x = 0
        cmd.angular.z = 0
        self.cmd_pub.publish(cmd) # stop the robot

    def robot_collided(self):
        #! Check if the robot is collided with an obstacle
        return not self.svc.is_valid(self.current_pose[0:2])
    
    def publish_frontiers(self):
        rospy.loginfo("Number of frontiers: %d", len(self.frontiers))
        marker_array = MarkerArray()

        marker = Marker()
        marker.header.frame_id = self.current_gridmap.header.frame_id
        marker.type = marker.POINTS
        marker.action = marker.ADD
        marker.id = 0

        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0

        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        for frontier in self.frontiers:
            point = Point()
            frontier = self.svc.__map_to_position__(np.array(frontier))
            point.x = frontier[0]
            point.y = frontier[1]
            point.z = 0.0
            marker.points.append(point)

        marker_array.markers.append(marker)

        self.frontier_pub.publish(marker_array)


    def publish_centroids(self):

        marker = Marker()
        marker.header.frame_id = self.current_gridmap.header.frame_id # Marker's reference frame
        marker.header.stamp = rospy.Time.now()
        marker.ns = "centroids"  
        marker.id = 0  # Marker ID
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD  
        marker.scale.x = 0.15  
        marker.scale.y = 0.15  
        marker.scale.z = 0.15 
        marker.color.a = 1.0  
        marker.color.r = 0.0 
        marker.color.g = 0.0 
        marker.color.b = 1.0 
        marker.lifetime = rospy.Duration()  
        marker.pose = Pose()

        # Create sphere points
        sphere_points = []
        for i, vp in enumerate(self.centroids):
            vp = self.svc.__map_to_position__(np.array(vp))
            point = Point()
            point.x = vp[0]  
            point.y = vp[1]  
            point.z = 0.0  
            sphere_points.append(point)

        marker.points = sphere_points
        self.centroid_pub.publish(marker)
        rospy.loginfo("Published centroids")


    def publish_nodes(self):

        marker = Marker()
        marker.header.frame_id = self.current_gridmap.header.frame_id # Marker's reference frame
        marker.header.stamp = rospy.Time.now()
        marker.ns = "nodes"  
        marker.id = 0  # Marker ID
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD  
        marker.scale.x = 0.15  
        marker.scale.y = 0.15  
        marker.scale.z = 0.15 
        marker.color.a = 1.0  
        marker.color.r = 1.0 
        marker.color.g = 0.0 
        marker.color.b = 0.0 
        marker.lifetime = rospy.Duration()  
        marker.pose = Pose()

        # Create sphere points
        sphere_points = []
        for i, vp in enumerate(self.nodes):
            point = Point()
            point.x = vp[0]  
            point.y = vp[1]  
            point.z = 0  
            sphere_points.append(point)

        marker.points = sphere_points
        self.node_pub.publish(marker)


    def publish_edges(self):
        marker = Marker()
        marker.header.frame_id = self.current_gridmap.header.frame_id
        marker.type = marker.LINE_LIST
        marker.action = marker.ADD
        marker.scale.x = 0.02
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        for i in range(len(self.nodes)):
            p1 = Point()
            p1.x = self.nodes[i][0]
            p1.y = self.nodes[i][1]
            p2 = Point()
            p2.x = self.nodes[self.parent[i]][0]
            p2.y = self.nodes[self.parent[i]][1]
            marker.points.append(p1)
            marker.points.append(p2)
        self.edge_pub.publish(marker)

    def publish_bound(self):
        if self.bounds and self.current_gridmap:
            marker = Marker()
            marker.header.frame_id = self.current_gridmap.header.frame_id
            marker.type = marker.LINE_LIST
            marker.action = marker.ADD
            marker.scale.x = 0.02
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 1.0
            
            p1 = Point()
            p1.x = self.bounds[0]
            p1.y = self.bounds[2]
            

            p2 = Point()
            p2.x = self.bounds[0]
            p2.y = self.bounds[3]
            

            p3 = Point()
            p3.x = self.bounds[1]
            p3.y = self.bounds[3]
            
            p4 = Point()
            p4.x = self.bounds[1]
            p4.y = self.bounds[2]

            marker.points.append(p1)
            marker.points.append(p2)
            marker.points.append(p2)
            marker.points.append(p3)
            marker.points.append(p3)
            marker.points.append(p4)
            marker.points.append(p4)
            marker.points.append(p1)

            self.bounds_pub.publish(marker)

    def visualize(self,a):

        self.publish_bound()
        if self.nodes and self.parent:
            self.publish_edges()
            self.publish_nodes()
            

    def discretize_segment(self,start, end):
        result_path = Path()
        result_path.header.frame_id = self.current_gridmap.header.frame_id
        result_path.header.stamp = rospy.Time.now()

        # Extract start and end coordinates
        incremental_range = 0.05
        x_start, y_start = start[0:2]
        x_end, y_end = end[0:2]
        total_distance = np.linalg.norm(np.array([x_start,y_start]) - np.array([x_end,y_end]))

        # Calculate the number of waypoints
        num_points = int(total_distance / incremental_range) + 1

        # Calculate the increments for each axis
        dx = (x_end - x_start) / (num_points - 1)
        dy = (y_end - y_start) / (num_points - 1)

        # Generate the waypoints
        for i in range(num_points):
            x = x_start + i * dx
            y = y_start + i * dy
            p = PoseStamped()
            p.pose.position.x = x
            p.pose.position.y = y
            result_path.poses.append(p)

        # print("start:{} end:{}".format(start[:2],end[:2]))
        # print("dis_path:{}".format(result_path))

        return result_path
    
    def discretize_path(self,path):
        result_path = Path()
        result_path.header.frame_id = self.current_gridmap.header.frame_id
        result_path.header.stamp = rospy.Time.now()

        # Extract start and end coordinates
        incremental_range = 0.05
        for i in range(len(path) -1):
            x_start, y_start = path[i]
            x_end, y_end = path[i + 1]
            total_distance = np.linalg.norm(np.array([x_start,y_start]) - np.array([x_end,y_end]))

            # Calculate the number of waypoints
            num_points = int(total_distance / incremental_range) + 1

            # Calculate the increments for each axis
            dx = (x_end - x_start) / (num_points - 1)
            dy = (y_end - y_start) / (num_points - 1)

            # Generate the waypoints
            for i in range(num_points):
                x = x_start + i * dx
                y = y_start + i * dy
                p = PoseStamped()
                p.pose.position.x = x
                p.pose.position.y = y
                result_path.poses.append(p)

        # print("start:{} end:{}".format(start[:2],end[:2]))
        # print("dis_path:{}".format(result_path))

        return result_path

# MAIN FUNCTION
if __name__ == '__main__':
    rospy.init_node('global_planner_node')   
    node = OnlinePlanner('/projected_map', '/odom', '/cmd_vel', [-10.0, 10.0, -10.0, 10.0], 0.2)
    
    # Run forever
    rospy.spin()


