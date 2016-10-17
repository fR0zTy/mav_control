#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Tue Oct  4 14:35:10 2016

@author: fr0zty
"""
import sys
import time, sys, math

try:
    import numpy as np
    from pyqt4 import QtCore
except ImportError as ie:
    print('One of the libraries missing',ie)
    sys.exit()
try:
    import rospy, tf
    from geometry_msgs.msg import Twist, Pose, Quaternion
    from gazebo_msgs.msg import ModelStates
    from std_srvs.srv import Empty
except ImportError:
    print('Import error on ROS, verify proper ROS installation')
    
class Base_Controller(object):
    
    def __init__(self):
        self.rospy_init()
        rospy.loginfo(rospy.get_name() + ' -- Initialization Complete!')
    
        
    def rospy_init(self, rate=10):
        
        rospy.init_node('keyboard_controller', anonymous=True)
        self.modal_states = rospy.Subscriber('/gazebo/model_states',ModelStates, self.get_model_states, queue_size=10)
        rospy.wait_for_service('/gazebo/reset.world')
        
        self.gz_reset = rospy.ServiceProxy('/gazebo/reset_world',Empty)
        
        self.quad_pose = Pose()
        self.quad_twist = Twist()
        self.rate = rospy.Rate(rate)
        
        
        self.msg = None

    def get_modelstates(self, data):
        quad_id = None
        for i, name in enumerate(data.name):
            if name == 'quadrotor':
                quad_id = i
                break
        if quad_id == None:
            return
            
        self.quad_pose = data.pose[quad_id]
        self.quad_twist = data.twist[quad_id]

    def get_twist(self, twist_velocity, key_events):
        
        dx, dy, dz = twist_velocity
        dax, day, daz = twist_velocity
        
        twist_events = {  #x, y, z, pitch, roll, yaw
                        QtCore.Qt.Key_W: [dx, 0, 0, 0, 0, 0],
                        QtCore.Qt.Key_S: [-dx, 0, 0, 0, 0, 0],
                        QtCore.Qt.Key_A: [0, dy, 0, 0, 0, 0],
                        QtCore.Qt.Key_D: [0, -dy, 0, 0, 0, 0], 
                        QtCore.Qt.Key_UP: [0, 0, dz, 0, 0, 0],
                        QtCore.Qt.Key_DOWN: [0, 0, -dz, 0, 0, 0],
                        QtCore.Qt.Key_LEFT: [0, 0, 0, 0, 0, daz],
                        QtCore.Qt.Key_RIGHT: [0, 0, 0, 0, 0, -daz],
                        }
        return twist_event[key_event]

                
    
    def reset(self):
        assert self.gz_reset()
        self.msg.linear.x = 0
        self.msg.linear.y = 0
        self.msg.linear.z = 0
        self.msg.angular.x  = 0
        self.msg.angular.y  = 0
        self.msg.angular.z  = 0
                        

        
                        
        
        