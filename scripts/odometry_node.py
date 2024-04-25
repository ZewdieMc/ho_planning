#!/usr/bin/python3
# -*- coding: utf-8 -*-

# Basic imports
import numpy as np
import math
import roslib
import rospy
import tf
# from tf.broadcaster import _
from tf.transformations import quaternion_from_euler, euler_from_quaternion

# ROS messages
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState, NavSatFix
from std_msgs.msg import Header, Float32MultiArray
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray



def wrap_angle(ang):
    if isinstance(ang, np.ndarray):
        ang[ang > np.pi] -= 2 * np.pi
        return ang
    else:
        return ang + (2.0 * math.pi * math.floor((math.pi - ang) / (2.0 * math.pi)))


class Robot:
    """
    Class that represents a Robot.
    """

    def __init__(self):
        # Robot physical parameters
        self.b = 0.23
        self.r = 0.035

        # Wheel velocities
        self.left_wheel_vel = 0.0
        self.right_wheel_vel = 0.0

        # Linear and angular speeds
        self.v = 0
        self.w = 0

        # Flags and time initialization
        self.left_wheel_received = False
        self.last_time = rospy.Time.now()

        # Robot position initialization
        self.x =2.15 
        self.y = 1.4
        self.th = -np.pi/2
        

        # Robot pose initialization
        self.xk = np.array([[self.x], [self.y], [self.th]])
        self.Pk = 2 * np.eye(3)


        # Model noise initialization
        self.right_wheel_noise_sigma = 0.2
        self.left_wheel_noise_sigma = 0.2
        self.Qk = np.array([[self.right_wheel_noise_sigma**2, 0],
                            [0, self.left_wheel_noise_sigma**2]])

        # Odom publisher and subscriber initialization
        self.sub = rospy.Subscriber(
            '/turtlebot/joint_states', JointState, self.joint_state_callback)
        
        self.odom_pub = rospy.Publisher(
            '/turtlebot/odom', Odometry, queue_size=10)

        self.tf_br = tf.TransformBroadcaster()

    def joint_state_callback(self, msg):
        if msg.name[0] == "turtlebot/kobuki/wheel_left_joint":
            self.left_wheel_vel = msg.velocity[0]
            self.left_wheel_rec = True

        elif msg.name[0] == "turtlebot/kobuki/wheel_right_joint":
            self.right_wheel_vel = msg.velocity[0]

            if self.left_wheel_rec:
                # calculation
                left_lin_vel = self.left_wheel_vel * self.r
                right_lin_vel = self.right_wheel_vel * self.r

                self.v = (left_lin_vel + right_lin_vel) / 2.0
                self.w = (left_lin_vel - right_lin_vel) / self.b

                # print("w ", self.w)

                # calculate dt
                current_time = rospy.Time.from_sec(
                    msg.header.stamp.secs + msg.header.stamp.nsecs * 1e-9)
                dt = (current_time - self.last_time).to_sec()
                # print("dt", dt)
                self.last_time = current_time

                # reset flag
                self.left_wheel_rec = False


                # Prediction step
                self.prediction(dt)
                # print("xk",self.xk)

                # Odom path publisher
                self.odom_path_pub()

   

    def prediction(self, dt):
        """
        Predicts the next state of the robot using the motion model.

        Args:
        - dt: The time interval between the current state and the next state.

        Returns:
        - None

        """

        # Calculate Jacobians with respect to state vector
        H = np.array([[1, 0, -math.sin(float(self.xk[2]))*(self.v)*dt],
                      [0, 1, math.cos(float(self.xk[2]))*(self.v)*dt],
                      [0, 0, 1]])

        # Calculate Jacobians with respect to noise
        Wk = np.array([[0.5 * math.cos(float(self.xk[2]))*dt, 0.5 * math.cos(float(self.xk[2]))*dt],
                       [0.5 * np.sin(float(self.xk[2]))*dt, 0.5 *
                        math.sin(float(self.xk[2]))*dt],
                       [-dt/self.b, dt/self.b]])

        # Update the prediction "uncertainty"
        self.Pk = H @ self.Pk @ np.transpose(H) + \
            Wk @ self.Qk @ np.transpose(Wk)

        # Integrate position
        self.xk[0] = self.xk[0] + math.cos(float(self.xk[2]))*(self.v)*dt
        self.xk[1] = self.xk[1] + math.sin(float(self.xk[2]))*(self.v)*dt
        self.xk[2] = wrap_angle(self.xk[2] + (self.w)*dt)


    def odom_path_pub(self):

        # Transform theta from euler to quaternion
        q = quaternion_from_euler(0, 0, float(wrap_angle(self.xk[2])))

        # Publish predicted odom
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = "world_ned"
        odom.child_frame_id = "turtlebot/base_footprint"

        odom.pose.pose.position.x = self.xk[0]
        odom.pose.pose.position.y = self.xk[1]

        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]

        odom.pose.covariance = [self.Pk[0, 0], self.Pk[0, 1], 0, 0, 0, self.Pk[0, 2],
                                self.Pk[1, 0], self.Pk[1,1], 0, 0, 0, self.Pk[1, 2],
                                0, 0, 0, 0, 0, 0,
                                0, 0, 0, 0, 0, 0,
                                0, 0, 0, 0, 0, 0,
                                self.Pk[2, 0], self.Pk[2, 1], 0, 0, 0, self.Pk[2, 2]]

        odom.twist.twist.linear.x = self.v
        odom.twist.twist.angular.z = self.w

        self.odom_pub.publish(odom)

        self.tf_br.sendTransform((float(self.xk[0]), float(self.xk[1]), 0.0), q, rospy.Time.now(
        ), odom.child_frame_id, odom.header.frame_id)


if __name__ == '__main__':

    rospy.init_node('Dead_reckoning')
    robot = Robot()

    rospy.spin()

# ===============================================================================


##################################################################################################