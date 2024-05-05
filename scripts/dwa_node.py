#!/usr/bin/python3

import rospy
from  nav_msgs.msg import Odometry, OccupancyGrid, GridCells
from geometry_msgs.msg import PoseWithCovarianceStamped, Point, Pose, Vector3, PoseStamped, Twist
from std_msgs.msg import ColorRGBA
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker
import numpy as np
import math
from numpy import cos, sin
from utils.state_validity_checker import StateValidityChecker
import tf
import actionlib
from ho_planning.msg import FollowPathAction, FollowPathFeedback, FollowPathResult

class DWANode:
    def __init__(self, gridmap_topic, odom_topic):
        # Tuning Parameters 
        self.dt = 2.0
        self.v_max = 0.2 # max linear vel
        self.v_min = 0.09 # min linear vel
        self.acc = 2.0 # linear acceleration

        self.w_max = 1.5 # max angular vel
        self.w_min = 0.1 # min angular vel
        self.acc_ang = 0.4 # angular acceleration
        # resolutions
        self.v_res = 0.05
        self.w_res = 0.05

        self.goal_heading_bias = 10.0
        self.clearance_bias = 1
        self.velocity_bias = 5
        self.distance_bias = 10

        self.window_size = 0.5 # m
        self.map_update_interval = 0.5
        self.enable_visualization = True
        self.ned = True
        # Class Module
        self.svc = StateValidityChecker(distance=0.25)   
            
        # Class Parameters
        self.goal = None
        self.path = None
        self.current_pose = [0,0,0]
        self.current_vel  = [0,0]
        self.last_map_time = rospy.Time.now()
        self.active = False
        self.action_rate = rospy.Rate(5)

        # ACTION SERVER
        self.server = actionlib.SimpleActionServer('follow_path', FollowPathAction, self.handle_follow_path, False)
        self.server.start()
        # PUBLISHER
        self.vel_pub = rospy.Publisher('/cmd_vel',Twist,queue_size=1)
        self.states_pub = rospy.Publisher('~states',Marker,queue_size=1)
        self.trajectory_pub = rospy.Publisher('~local_path',Marker,queue_size=1)
        self.closest_obs_pub = rospy.Publisher('~closest_obs',Marker,queue_size=1)

        # SUBSCRIBERS
        self.odom_sub = rospy.Subscriber(odom_topic,Odometry, self.odom_cb) 
        self.gridmap_sub = rospy.Subscriber(gridmap_topic,OccupancyGrid, self.gridmap_cb) 
        # self.path_sub = rospy.Subscriber('/global_path',Path,self.path_cb)
        # self.move_goal_sub = rospy.Subscriber('/waypoints', PoseStamped, self.get_goal)  # Only for testing purposes


        rospy.Timer(rospy.Duration(0.1), self.control_loop)
        
        self.closest_obs = None
        


    ##################################################################
    #### Callback functions
    ##################################################################
    def gridmap_cb(self, data):
        '''
        callback for gridmap
            - 
        '''
        if (data.header.stamp - self.last_map_time).to_sec() > self.map_update_interval:      
      
            self.last_map_time = data.header.stamp
            self.current_gridmap = data

            # Update State Validity Checker
            env = np.array(data.data).reshape(data.info.height, data.info.width).T
            origin = [data.info.origin.position.x, data.info.origin.position.y]
            self.svc.set(env, data.info.resolution, origin)

            

    def odom_cb(self,odom):
        _, _, yaw = tf.transformations.euler_from_quaternion([odom.pose.pose.orientation.x, 
                                                              odom.pose.pose.orientation.y,
                                                              odom.pose.pose.orientation.z,
                                                              odom.pose.pose.orientation.w])
        x = odom.pose.pose.position.x 
        y = odom.pose.pose.position.y
        
        self.current_pose = np.array([x,y,yaw])
        self.current_vel = [odom.twist.twist.linear.x, odom.twist.twist.angular.z]

    def get_goal(self, goal):
        rospy.loginfo("goal: %s", goal)
        self.goal = [goal.pose.position.x, goal.pose.position.y]
            
    def path_cb(self, msg):
        path = []
        for p in msg.poses:
            path.append([p.pose.position.x, p.pose.position.y])
        self.path = path

    def handle_follow_path(self, goal):
        self.reset_robot()
        success = True
        # Deal with goal
        self.path = []
        for p in goal.path.poses:
            self.path.append([p.pose.position.x, p.pose.position.y])
        
        self.active = True
        while not self.near_goal() and not rospy.is_shutdown():
            # if self.robot_is_stuck():
            #     rospy.logerr('Robot Stuck!!')
            #     self.reset_robot()
            #     self.server.set_preempted()
            #     success = False
            #     break
            
            if self.server.is_preempt_requested():
                rospy.logerr('Preemptted!!')
                self.reset_robot()
                self.server.set_preempted()
                success = False
                break
            
            # fb = FollowPathFeedback()
            # fb.current_path = P
            # self.server.publish_feedback()
            
            self.action_rate.sleep()
            
            
        
        self.reset_robot()
        if success:
            rospy.loginfo('Succeeded')
            result = FollowPathResult()
            result.success = True
            self.server.set_succeeded(result)

                




    def reset_robot(self):
        self.path = None
        self.goal = None
        self.active = False
        self.vel_pub.publish(Twist(Vector3(0,0,0), Vector3(0,0,0)))



    ##################################################################
    #### Alogorithm Functions
    ##################################################################
    def control_loop(self, _):
        if self.active:
            # try:    
            self.update_goal_from_path()
            if self.goal and self.svc.there_is_map:
                
                self.v_array,self.w_array = self.generate_velocity_array()
                v,w = self.compute_velocity(self.v_array,self.w_array)
                if self.ned:
                    self.vel_pub.publish(Twist(Vector3(v,0,0), Vector3(0,0,w)))
                    self.visualize()
                else:
                    self.vel_pub.publish(Twist(Vector3(v,0,0), Vector3(0,0,w)))
                    self.visualize()
            # except Exception as e:
            #     rospy.logerr(e)

    def generate_velocity_array(self ):
        current_vel = self.current_vel
        currnet_v_max = min(current_vel[0] + self.acc*self.dt, self.v_max)
        currnet_v_min = max(current_vel[0] - self.acc*self.dt, 0)

        vs = np.arange(currnet_v_min, currnet_v_max+ self.v_res, self.v_res)

        vs = np.round(vs,2)
        v_bond = np.where((vs == 0) | (vs >= self.v_min) | (vs <= -self.v_min))
        
        vs = vs[v_bond]
        currnet_w_max = min(current_vel[1] + self.acc_ang*self.dt, self.w_max)
        currnet_w_min = max(current_vel[1] - self.acc_ang*self.dt, -self.w_max)

        ws = np.arange(currnet_w_min, currnet_w_max + self.w_res, self.w_res)
        ws = np.round(ws,2)
        w_bond = np.where((ws == 0) | (ws >= self.w_min) | (ws <= -self.w_min) )
        ws = ws[w_bond]
            
        if ws.any() != 0:
            ws = list(ws)
            ws.append(0.0)
            ws.sort()

        return list(vs), list(ws)
    
    def compute_velocity(self,vs,ws):
        states = np.empty((len(ws), len(vs)), dtype=object)
        scores = np.ones((len(ws), len(vs))) * -1 # make initial scores < 0
        penalty_list = []
        x = self.current_pose

        current_max_v = max(vs)
        current_min_v = min(vs)

        for i, w in enumerate(ws):
            for j, v in enumerate(vs):
                xi, yi, theta_i = self.compute_pose(v, w, x[0], x[1], x[2], self.dt)
                state = [xi, yi, theta_i, v, w]
                states[i,j] = state

                score = self.goal_heading_bias * self.heading_score(state) + \
                        self.clearance_bias * self.clearance_score(state) +\
                        self.velocity_bias * self.velocity_score(state,current_max_v,current_min_v)# +\
                        # self.distance_bias * self.dist(state)
                # prevent selecting 0
                if v == 0.0:
                    score = 0
                

                if score >= 0 or v == 0:
                    scores[i,j] = score
                else:
                    penalty_list.append(i)

        unique_penalty_list =  list(set(penalty_list))
        for i in unique_penalty_list:
            scores[i,:] = -9999
            
        self.states = states
        self.scores = scores

        #! update self.scores to account for the distance to the closest 
        self.dist_score()

        max_idx = np.argmax(self.scores)
        i, j = np.unravel_index(max_idx, scores.shape)
        if not( i == 0 and j == 0):
            self.selected_states = self.states[i, j] 
        else:
            self.selected_states = [0,0,0,0,0] # stop the robot if every state is invalid

        return self.selected_states[3], self.selected_states[4]
    
    def compute_pose(self,v, w, x, y, theta, dt):
        theta_i = theta + w * dt
        xi = x + v * np.cos(theta_i) * dt
        yi = y + v * np.sin(theta_i) * dt   
        return xi, yi, theta_i

    def dist_score(self):
        obs_states = []
        fine_states = []
        distances = []
        
        for i in range(self.states.shape[0]):
            for j in range(self.states.shape[1]):
                state = self.states[i, j]
                #
                if self.scores[i, j] < 0:
                    obs_states.append(state)
                else:
                    fine_states.append(state)
        if len(obs_states) > 0:
            for obs_state in obs_states:
                distance = self.distance_to_robot(np.array(obs_state[:2]))
                distances.append([obs_state[0], obs_state[1], distance])
            
            closest_obs = max(distances, key=lambda x: x[2])
            self.closest_obs = closest_obs
            
            for i, s in enumerate(self.states):
                for j, element in enumerate(s):

                    if np.all(self.scores[i, :] < 0):
                        print("obstacle states")
                        continue
                    else:
                        dist_to_closest_obs = np.linalg.norm(np.array(element[:2]) - np.array(closest_obs[:2]))
                        self.scores[i, :] =  self.scores[i, :] + dist_to_closest_obs/ self.distance_bias


    def heading_score(self,point):
        #distance = ((goal[0] - point[0])**2 + (goal[1] - point[1])**2)**0.5
        heading_score = 0
        if self.goal:    
            psi_d = np.arctan2(self.goal[1] - point[1], self.goal[0] - point[0])
            psi = wrap_angle(psi_d - point[2])
            # worst is +pi or -pi, best is 0. Give scores between 0-1 using these as min and max. 
            heading_score = 1 - abs(psi)  / np.pi
            
        return heading_score
    
    def clearance_score(self,point):
        if not self.svc.is_valid(point[0:2]):
            clearance_score = -999
        #I want to score -999 for points very close to point as well
        else:
            clearance_score = 1
            #invalid_points = get_invalid_points()
        
            # for invalid_point in invalid_points:
            #     distance = ((invalid_point[0] - point[0])**2 + (invalid_point[1] - point[1])**2)**0.5
            #     if distance < 2:
            #         return -999
                
        return clearance_score

    def velocity_score(self,point, max_linear_velocity, min_linear_velocity):
        #worst is 0, best is 1
        velocity_score = (abs(point[3]) - min_linear_velocity) / (max_linear_velocity - min_linear_velocity)
        return velocity_score 
    
    def update_goal_from_path(self):
        if self.path:
            idx = 0
            for i, p in enumerate(self.path):
                dis = self.distance_to_robot(p)
                self.goal = p
                idx = i
                if dis > self.window_size:    
                    break
            # Update path
            self.path = self.path[idx:]
        
    ##################################################################
    #### Visualize functions
    ##################################################################
    def visualize(self):
        if self.enable_visualization:
            self.visualize_states()
            self.visualize_trajectories()
            self.publish_closest_obs()

    

    def visualize_states(self):
        
        marker = Marker()
        marker.header.frame_id = self.current_gridmap.header.frame_id  # Marker's reference frame
        marker.header.stamp = rospy.Time.now()
        marker.ns = "viewpoint"  # Namespace to avoid conflicts
        marker.id = 0  # Marker ID
        marker.type = Marker.SPHERE_LIST  # Type of marker (SPHERE_LIST)
        marker.action = Marker.ADD  # Action to take (ADD, DELETE, etc.)
        marker.scale.x = 0.05  # Scale (diameter along x-axis)
        marker.scale.y = 0.05  # Scale (diameter along y-axis)
        marker.scale.z = 0.0  # Scale (diameter along z-axis)
        marker.color.a = 1.0  # Alpha (transparency)
        marker.color.r = 0.0  # Red component
        marker.color.g = 0.0  # Green component
        marker.color.b = 1.0  # Blue component
        marker.lifetime = rospy.Duration()  # Marker lifetime (0 means forever)
        marker.pose = Pose()

        # Create some example sphere points
        sphere_points = []
        colors = []
        states = list(self.states.flatten())
        scores = list(self.scores.flatten())
        for i, state in enumerate(states):
            point = Point()
            point.x = state[0]  # Example x-coordinate
            point.y = state[1]  # Example y-coordinate
            point.z = 0.0  # Example z-coordinate
            sphere_points.append(point)
            if scores[i] < 0:
                color = ColorRGBA(r=1, g=0, b=0, a=1)
            else:
                color = ColorRGBA(r=0, g=1, b=0, a=1)
            colors.append(color)

        marker.points = sphere_points
        marker.colors = colors
        self.states_pub.publish(marker)


    def publish_closest_obs(self):
        if self.closest_obs is not None:    
            marker = Marker()
            marker.header.frame_id = "odom"
            marker.type = marker.SPHERE
            marker.action = marker.ADD

            # x, y = self.__map_to_position__(f,res,orig)
            # print("aaaaaa", x ,y)
            marker.header.stamp = rospy.Time.now()
            marker.pose.position.x = self.closest_obs[0]
            marker.pose.position.y = self.closest_obs[1]
            
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1

            marker.color.g = 1
            marker.color.r = 0
            marker.color.b = 0.7
            marker.color.a = 1

            self.closest_obs_pub.publish(marker)
        else:
            rospy.logwarn("closest_obs is None")

    def visualize_trajectories(self):
        marker_msg = Marker()
        marker_msg.header.stamp = rospy.Time.now()
        marker_msg.header.frame_id = self.current_gridmap.header.frame_id  # Specify the frame ID
        marker_msg.type = Marker.LINE_STRIP
        marker_msg.action = Marker.ADD
        marker_msg.pose.orientation.w = 1.0  # Default orientation

        marker_msg.scale.x = 0.03  # Line width

        marker_msg.color.b = 1.0  # Red color
        marker_msg.color.a = 1.0  # Full opacity

        path = self.find_path(self.selected_states)
        for p in path:
            point = Point()
            point.x = p[0]
            point.y = p[1]
            point.z = 0.0  # Assuming 2D trajectory, adjust for 3D if needed
            marker_msg.points.append(point)

        # Publish the Marker message
        self.trajectory_pub.publish(marker_msg)

    def find_path(self,state):
        path = []
        t = [i * self.dt/10 for i in range(10)]

        for dt in t:
            x, y, _ = self.compute_pose(state[3],state[4],self.current_pose[0],self.current_pose[1],self.current_pose[2],dt)
            path.append([x,y])
    
        return path
    ##################################################################
    #### Utility functions
    ##################################################################
    def distance_to_robot(self,p):
        return np.linalg.norm(np.array(self.current_pose[:2] -p) )
    
    def near_goal(self):
        return np.linalg.norm(np.array(self.current_pose[:2] -self.path[-1]) ) < 0.15


def wrap_angle(angle):
    return (angle + ( 2.0 * np.pi * np.floor( ( np.pi - angle ) / ( 2.0 * np.pi ) ) ) )


if __name__ == '__main__':

    rospy.init_node('dwa')
    robot = DWANode("/projected_map", "/odom")
    rospy.loginfo("DWA node started")

    rospy.spin()
