#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import Twist
import smach
from visualization_msgs.msg import Marker, MarkerArray
from utils.global_planner import StateValidityChecker
import tf
import smach_ros

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Pose
from std_msgs.msg import ColorRGBA 
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from utils.global_planner import StateValidityChecker
import actionlib
from ho_planning.msg import FollowPathAction, FollowPathGoal, FollowPathResult
from ho_planning.srv import GetGlobalPlan, GetGlobalPlanRequest, GetGlobalPlanResponse


global black_board

class IDLE(smach.State):
    '''
    State for waiting for command
    '''
    def __init__(self):
        global black_board
        smach.State.__init__(self, outcomes=['goal_recieved','start_exploration','state_end'])

    def execute(self, userdata):
        global black_board
        rospy.loginfo('Executing state IDLE')
        black_board.reset_flags()
        black_board.stop_robot()

        while not rospy.is_shutdown():
            if black_board.movebase_recieved:
                rospy.loginfo('Move base goal received --> start MOVEBASE')

                return 'goal_recieved'
            elif black_board.start_exploration:
                rospy.loginfo('Exploration trigger received --> start EXPLORATION')
                return'start_exploration'
            rospy.sleep(1)

        return 'state_end'


class FindPath(smach.State):
    '''
    State for finding global paths
    '''
    def __init__(self):
        global black_board
        smach.State.__init__(self, outcomes=['path_found','plan_fail'])
        self.retries = 5

    def execute(self, userdata):
        global black_board
        rospy.loginfo('Executing state FIND_PATH')
        black_board.path = None
        black_board.stop_robot()

        # Call global planner service
        for i in range(self.retries): 
            black_board.path = black_board.get_global_plan()
            if black_board.path:
                rospy.loginfo('Path found')
                return 'path_found'
            
        return 'plan_fail'
    
class FollowPath(smach.State):
    def __init__(self):
        global black_board
        smach.State.__init__(self, outcomes=['goal_reached','robot_stuck','invalid_path'])
    

    def execute(self, userdata):
        global black_board
        rospy.loginfo('Executing state FOLLOW_PATH')
        black_board.stop_robot()

        # Send command through action server
        black_board.send_action()
        while black_board.path and not rospy.is_shutdown():
            if black_board.client.get_state() == actionlib.GoalStatus.PREEMPTED :
                rospy.logerr('Robot Stuck')
                black_board.client.cancel_goal()
                black_board.stop_robot()
                black_board.path = None
                return 'robot_stuck'
            
            if black_board.client.get_state() == actionlib.GoalStatus.SUCCEEDED:
                rospy.loginfo('Goal Reached')
                black_board.stop_robot()
                black_board.path = None
                return 'goal_reached'

            rospy.sleep(0.5)
            
        rospy.logwarn('Path not valid anymore')
        black_board.stop_robot()
        black_board.client.cancel_goal()
        black_board.path = None
        return 'invalid_path'
    
class Recovering(smach.State):
    def __init__(self):
        global black_board
        smach.State.__init__(self, outcomes=['recover_success','recover_fail'])
        
    

    def execute(self, userdata):
        global black_board
        rospy.loginfo('Executing state RECOVERING')
        black_board.stop_robot()
        black_board.back_off()

        if black_board.svc.is_valid(black_board.current_pose[:2]):
            rospy.loginfo('Robot is valid')
            return'recover_success'
        else:
            rospy.logerr('Robot is still stuck')
            return'recover_fail'

        

        
class StateBlackBoard:
    '''
    Class to contain all tools and shared variables for state machine
    '''
    def __init__(self):
        # Shared Variable
        self.goal = None
        self.nbvp = None
        self.path = None
        self.current_pose = None
        
        # Flags
        self.movebase_recieved = False
        self.start_exploration = False
        

        # Grid map
        self.svc = StateValidityChecker(0.2)
        self.current_gridmap = None
        self.last_map_time = rospy.Time.now()

        # Vel Publisher
        self.vel_pub = rospy.Publisher('/cmd_vel',Twist, queue_size=10)

        # Path marker
        self.marker_pub = rospy.Publisher('~path_marker', Marker, queue_size=1)

        # Subscribers
        #?subscriber to gridmap_topic from Octomap Server  
        self.gridmap_sub = rospy.Subscriber('/projected_map', OccupancyGrid, self.get_gridmap)
        #?subscriber to odom_topic  
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.get_odom)
        # ?subscriber to /move_base_simple/goal published by rviz
        self.move_goal_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.get_goal)    

        # ACTION CLIENT
        self.client = actionlib.SimpleActionClient('follow_path', FollowPathAction)
        rospy.loginfo("Waiting for DWA Server to connect")
        self.client.wait_for_server()

        # Visualization
        rospy.Timer(rospy.Duration(2),self.visualization)

    #######################
    ### Callbacks
    #######################
        
    def get_odom(self, odom):
        _, _, yaw = tf.transformations.euler_from_quaternion([odom.pose.pose.orientation.x, 
                                                              odom.pose.pose.orientation.y,
                                                              odom.pose.pose.orientation.z,
                                                              odom.pose.pose.orientation.w])

        #?Store current position (x, y, yaw) as a np.array in self.current_pose var.
        self.current_pose = np.array([odom.pose.pose.position.x, odom.pose.pose.position.y, yaw])

    def get_goal(self, msg):
        if self.svc.there_is_map:
            #?Store goal (x,y) as a numpy aray in self.goal var and print it 
            goal = np.array([msg.pose.position.x, msg.pose.position.y])
            rospy.loginfo("New goal received: ".format(goal))
            if self.svc.is_valid(goal):
                self.goal = goal
                self.movebase_recieved = True
            else:
                self.goal = None
                self.movebase_recieved = False
                rospy.logerr("Goal is not valid")

    def get_gridmap(self, gridmap):
      
        # To avoid map update too often (change value '1' if necessary)
        if (gridmap.header.stamp - self.last_map_time).to_sec() > 2:            
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
            
            if self.path is not None:
                if not self.svc.check_path(self.path):
                    rospy.logerr("Path not valid anymore")
                    self.path = None 

    ###############################
    ### Service caller
    ###############################
    def get_global_plan(self):
        rospy.logwarn("Calling get global plan")
        rospy.wait_for_service('get_global_plan')
        path = []
        try:
            get_plan = rospy.ServiceProxy('get_global_plan', GetGlobalPlan)
            req = GetGlobalPlanRequest()
            req.goal_x = self.goal[0]
            req.goal_y = self.goal[1]

            resp = get_plan(req)
            # rospy.logwarn("matching: {}".format(resp.transform))
            if not resp.success:
                return None
            for i in range(len(resp.path_x)):
                path.append(np.array([resp.path_x[i], resp.path_y[i]]))

            path.insert(0, [self.current_pose[0], self.current_pose[1]])

            return path
        
        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")
            return None



    ###############################
    ### Actions
    ###############################
    def send_action(self):
        goal = FollowPathGoal()
        print(self.path)
        path_msg = self.discretize_path(self.path)
        goal.path = path_msg
        
        self.client.send_goal(goal)

    ###############################
    ### Robot Control
    ###############################
    def back_off(self):
        #! Move the robot back a little bit
        backoff_t1 = rospy.Time.now()
        cmd = Twist()
        cmd.linear.x = -0.1
        cmd.linear.y = 0
        cmd.linear.z = 0
        cmd.angular.x = 0
        cmd.angular.y = 0
        cmd.angular.z = 0
        rospy.logwarn("Moving the robot back")
        while (rospy.Time.now() - backoff_t1) < rospy.Duration(2):
            self.vel_pub.publish(cmd) # move the robot back
        cmd.linear.x = 0
        cmd.angular.z = 0
        self.vel_pub.publish(cmd) # stop the robot

    def stop_robot(self):
        cmd = Twist()
        cmd.linear.x = 0
        cmd.linear.y = 0
        cmd.linear.z = 0
        cmd.angular.x = 0
        cmd.angular.y = 0
        cmd.angular.z = 0
        
        self.vel_pub.publish(cmd) # stop the robot

    ###############################
    ### Utilities
    ###############################
    def reset_flags(self):
        self.movebase_recieved = False
        self.start_exploration = False

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
        return result_path
    
    def visualization(self,_):
        self.publish_path()

    def publish_path(self):
        if self.path:
            # print("Publish path!")
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

            
            for n in self.path:
                p = Point()
                p.x = n[0]
                p.y = n[1]
                p.z = 0.0
                m.points.append(p)
                m.colors.append(color_red)
            
            self.marker_pub.publish(m)

        


# main
def main():
    global black_board
    rospy.init_node('state_machine_node')
    rospy.loginfo('Starting State Machine')
    black_board = StateBlackBoard()
    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['state_end'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('IDLE', IDLE(), 
                               transitions={'goal_recieved':'MOVEBASE', 
                                            'start_exploration':'state_end',
                                            'state_end':'state_end'})
        movebase_sm = smach.StateMachine(outcomes=['movebase_end'])
        with movebase_sm:
            smach.StateMachine.add('FIND_PATH', FindPath(), 
                               transitions={'path_found':'FOLLOW_PATH', 
                                            'plan_fail':'movebase_end'})
            smach.StateMachine.add('FOLLOW_PATH', FollowPath(), 
                               transitions={'goal_reached':'movebase_end', 
                                            'robot_stuck':'RECOVERING',
                                            'invalid_path':'FIND_PATH'})
            smach.StateMachine.add('RECOVERING', Recovering(), 
                               transitions={'recover_success':'FIND_PATH', 
                                            'recover_fail':'movebase_end'})
            

        # explo_sm = smach.StateMachine(outcomes=['explo_end'])
        
        # with explo_sm:
        #     smach.StateMachine.add('FIND_NBVP', FindNbvp(black_board), 
        #                        transitions={'nbvp_found':'FIND_PATH', 
        #                                     'nbvp_fail':'explo_end'})
        #     smach.StateMachine.add('FIND_PATH', FindPath(black_board), 
        #                        transitions={'path_found':'FOLLOW_PATH', 
        #                                     'plan_fail':'FIND_NBVP'})
        #     smach.StateMachine.add('FOLLOW_PATH', FollowPath(black_board), 
        #                        transitions={'goal_reach':'movebase_end', 
        #                                     'robot_stuck':'RECOVERING',
        #                                     'invalid_path':'FIND_PATH'})
        #     smach.StateMachine.add('RECOVERING', Recovering(black_board), 
        #                        transitions={'recover_success':'FIND_PATH', 
        #                                     'recover_fail':'movebase_end'})
            
        smach.StateMachine.add('MOVEBASE', movebase_sm, 
                               transitions={'movebase_end':'IDLE'})
        # smach.StateMachine.add('EXPLORATION', explo_sm, 
        #                        transitions={'explo_end':'IDLE'})
            

    # Execute SMACH plan
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()
    outcome = sm.execute()
    rospy.spin()


if __name__ == '__main__':
    main()