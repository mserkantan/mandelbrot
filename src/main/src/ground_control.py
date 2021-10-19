#!/usr/bin/env python3

import rospy
from std_msgs.msg import Header, Bool
from geometry_msgs.msg import PoseStamped, PoseArray, TwistStamped

class GroundControl:
    def __init__(self) -> None:
        '''
        This is the driver part of the mock program (for description purposes, brainstorming etc.)

        Ground Control is responsible for obtaining necessary info to show the end user.

        For example, A Video feed from the Robot's camera,

        As well, the robot shall be externally controllable from this interface.

        The single/multi-threaded communication handled through ROS nodes/topics.

        Based on the subscriber/advertiser(publisher) methodology.
        
        '''

        rospy.init_node("ground_control") # The root of a communcation network (Node)
        
        self.rate = rospy.Rate(5) # Run the callbacks at 5Hz.

        '''Subscribers'''

        self.robotHealth = rospy.Subscriber("/heartbeat", Bool, self.robotHealthCallback)

        self.robotPos = rospy.Subscriber("/navsat/fix", PoseStamped, self.robotPosCallback)

        self.robotVel = rospy.Subscriber("/navsat/vel", TwistStamped, self.robotVelCallback)

        self.cameraFeed = rospy.Subscriber("/realsense/color/image_raw", TwistStamped, self.robotVelCallback)


        #self.rangefinder
        #self.humidity
        # 
        #etc. . .

        '''Publishers'''

        self.setRobotPosPub = rospy.Publisher("/goal", PoseStamped, queue_size=1)

        self.setRobotPathPub = rospy.Publisher("/path", PoseArray, queue_size=1)


        '''Class Instances'''
        self.robot_health = None
        self.robot_pos = None
        self.robot_vel = None


    '''Callbacks (Called every 0.2 secs.)'''

    def robotHealthCallback(self, data):
        '''Robot Health Status (Extendible)'''
        rospy.loginfo_throttle(5, data) # Every 5 seconds report the robot's health
        self.robot_health = data

    def robotPosCallback(self, data):
        '''Robot Position Record Update'''
        self.robot_pos = data
    
    def robotVelCallback(self, data):
        '''Robot Velocity Record Update'''
        self.robot_vel = data

    #TODO: Add a Camera Interface 
    #TODO: Add a HTTP interface (Flask etc.)

    '''============================================'''
    '''Running the actual robot FOR SITL TESTS ONLY'''
    '''============================================'''
    #Draw a Square based on local position

    def boot(self):
        pass