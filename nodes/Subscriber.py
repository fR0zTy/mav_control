#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
@author: fr0zty
"""
import sys
try:
    import rospy
    #import geometry_msgs.msg
except ImportError:
    print('Import Error: Check ROS installation')
    sys.exit()



class SubscriberNode(object):
    '''
    Default Subscriber class, for code reuse, use inheritance over this class, use method overloading for callback method \n
    
    Usage:
        
    >>>subscriber_obj = SubscriberNode(node_name, subscribed_topic_name, msg_type_object)  # Initialize subscriber parameters \n
    >>>subscriber_obj.subscribe()       #run the subscriber
    '''
    
    def __init__(self, node_name, topic_name, msg_obj):
        
        self.node_name = node_name
        self.topic_name = topic_name
        self.msg_obj = msg_obj
        
    def subscribe(self):
        
        rospy.loginfo("-- Initialising Subscriber--")

        rospy.init_node(self.node_name, anonymous=True)
        rospy.Subscriber(self.topic_name, self.msg_obj , self.callback)
        rospy.spin()
    
    @staticmethod
    def callback(data):
        raise NotImplementedError('Callback method for unit subscriber class, use Inheritance overloading')
        
