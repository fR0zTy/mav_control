#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
@author: fr0zty
"""
try:
    import rospy
    #import geometry_msgs.msg
except ImportError:
    print('import error on ROS package, verify ROS installation')



class SubscriberNode(object):
    '''
    Default Subscriber class, for code reuse, use inheritance overloading for callback method \n
    
    Usage:
        
    >>>SubscriberNode(node_name, subscribed_topic_name, msg_type_object)
    '''
    
    def __init__(self, node_name, topic_name, msg_obj):
        
        self.node_name = node_name
        self.topic_name = topic_name
        self.msg_obj = msg_obj
        
    def subscribe(self):
        rospy.loginfo("-- Importing msgs--")

        rospy.init_node(self.node_name, anonymous=True)
        rospy.Subscriber("%s"%self.topic_name, self.msg_obj , self.callback)
        rospy.spin()
    
    @staticmethod
    def callback(data):
        raise NotImplementedError('Callback method for unit subscriber class, use Inheritance overloading')
        
