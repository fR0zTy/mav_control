#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Tue Oct 25 16:01:41 2016

@author: fr0zty
"""
import sys
try:
    import rospy
except ImportError:
    print('Import Error: Check ROS installation')
    sys.exit()
    
    
class PublisherNode(object):
    '''
    Default Publisher class, for code reuse, use inheritance over this class \n
    
    Usage:
        
    >>>publisher_obj = PublisherNode(self, node_name, topic_name, msg, msg_obj, queue_size, rate)  # Initialize publisher parameters \n
    >>>publisher_obj.publish()      # run the publisher
    
    '''
    
    def __init__(self, node_name, topic_name, msg, msg_obj, queue_size, rate):
        
        self.node_name = node_name
        self.topic_name = topic_name
        self.msg = msg
        self.msg_obj = msg_obj
        self.queue_size = queue_size
        self.rate = rate
    
    def publish(self):
        
        rospy.loginfo("--- Initializing publisher ---")
        self.publisher = rospy.Publisher(self.topic_name, self.msg_obj, queue_size=self.queue_size )
        rospy.init_node(self.node_name, anonymous=True)
        rate = rospy.Rate(self.rate)
       
        while not rospy.is_shutdown():
            rospy.loginfo(self.msg)
            self.publisher.publish(self.msg)
            rate.sleep()   
