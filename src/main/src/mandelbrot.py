#!/usr/bin/env python


'''
@Author: Serkan Tan
October, 2021

mandelbrot.py 

This driver is used to maintain the robot's logic for example, 

- Running the Computer Vision Algos
- Handling Localization and Mapping Tasks
- Running Control Algorithms, EKF, PID etc.
- Advertising Status Info to Ground Control
- and anything related to the Robot: Sensors, Point Cloud Data etc.


'''

import rospy
from std_msgs.msg import Header, Bool
from geometry_msgs.msg import Twist, PoseWithCovariance, TwistWithCovariance, Pose
from sensor_msgs.msg import NavSatFix, NavSatStatus, Image
from nav_msgs.msg import Odometry
import numpy as np

class Mandelbrot:
    def __init__(self, id = ""):
        ''''''
        self.gps_pos = None
        self.pos = None
        self.vel = None
        self.curr_frame = None
        self.heartbeat = False
        self.id = id # In case of driving multiple Lawn Mowers. . . 


        self._should_exit = False

        #Initialize

        rospy.init_node("mandelbrot") # The root of a communcation network (Node)
        
        self.rate = rospy.Rate(1) # Run the callbacks at 20Hz.

        '''Subscribers'''

        self.robotPos = rospy.Subscriber("{}/navsat/fix".format(self.id), NavSatFix, self.robotPosCallback)

        self.robotOdometry = rospy.Subscriber("{}/husky_velocity_controller/odom".format(self.id), Odometry, self.robotOdomCallback)

        self.cameraFeed = rospy.Subscriber("{}/realsense/color/image_raw".format(self.id), Image, self.robotCameraFeedCallback)

        '''Publishers'''

        self.status_pub = rospy.Publisher("{}/heartbeat".format(self.id), Bool, queue_size=10)

        self.vel_pub = rospy.Publisher("{}/husky_velocity_controller/cmd_vel", Twist, queue_size=10)

    def robotPosCallback(self, data):
        '''Robot Position Record Update'''
        self.gps_pos = data

    def robotOdomCallback(self, data):
        self.pos = data.pose.pose.position
        self.vel = data.twist.twist

    def robotCameraFeedCallback(self, data):
        self.curr_frame = data

    def loop(self):

        self.heartbeat = True
        latitude  = None
        longitude = None
        altitude  = None
        localpos    = np.array([0.0, 0.0, 0.0])
        vel_linear  = np.array([0.0, 0.0, 0.0])
        vel_angular = np.array([0.0, 0.0, 0.0])

        while (not rospy.is_shutdown() and not self._should_exit):
            rospy.loginfo("ITERATION {}".format(rospy.Time.now()))
            if not self.gps_pos is None: 
                latitude = self.gps_pos.latitude
                longitude = self.gps_pos.longitude
                altitude = self.gps_pos.altitude
                rospy.loginfo(" position:= latitude: {} longitude: {} altitude: {}".format(latitude, longitude, altitude))
            if not self.pos is None:
                localpos[0], localpos[1], localpos[2] = self.pos.x, self.pos.y, self.pos.z
                rospy.loginfo("local position := {}".format(localpos))
            if not self.vel is None: 
                vel_linear[0], vel_linear[1], vel_linear[2] = self.vel.linear.x, self.vel.linear.y, self.vel.linear.z
                vel_angular[0], vel_angular[1], vel_angular[2] = self.vel.angular.x, self.vel.angular.y, self.vel.angular.z
                rospy.loginfo("linear velocity := {}".format(vel_linear))
                rospy.loginfo("angular velocity := {}".format(vel_angular))


            self.rate.sleep()

        if self._should_exit:
            self.heartbeat = False

if __name__ == "__main__":
    robot = Mandelbrot()
    robot.loop()