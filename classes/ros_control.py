#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
@author: fr0zty
"""
try:
    import rospy
    import rosgraph
    import rospkg
except ImportError as ie:
    print('Check ROS Installation', ie)

try:    
    import os
    import subprocess
    import json
    import sys
    import time
    import socket
    import warnings

except ImportError as ie:
    print('Import Libraries missing, install the specified library',ie)
    sys.exit()
#__________________________________________________________________________________________________

#------------------------------------------ROS Class-----------------------------------------------
#__________________________________________________________________________________________________

class ROS_Control(object):
    '''
    Python class for handling basic ROS Communications
    '''    
    def __init__(self):
        
        self.is_online = False
        self.ROS_MASTER_URI = ''
        self.ROS_PACKAGE_PATH = ''
        self.ROS_HOME = ''
        
#__________________________________________________________________________________________________
    
    def check_rosmaster(self):
    
        try:
            rosgraph.Master('/rostopic').getPid()
            print('rosmaster already running!')
            self.is_online = True
        except socket.error:
            warnings.warn('Unable to communicate with the master!')
            self.is_online = False
            
        
#__________________________________________________________________________________________________


    def kill_rosmaster(self):
        if self.is_online:
            cmd = subprocess.Popen(['killall roscore'], stdout=subprocess.PIPE, shell=True)
            out, err = cmd.communicate()
            return out, err
            sys.exit()
        else:
            print('rosmaster already offline')
    
        
#__________________________________________________________________________________________________
            
    def get_package_path(self):
        return self.ROS_PACKAGE_PATH
        
    def get_env(self):
        self.ROS_PACKAGE_PATH = rospkg.get_ros_paths()
        self.ROS_HOME = rospkg.get_ros_root()
        self.ROS_MASTER_URI = rospkg.get_ros_home()                   
        
    def kill_roscore(self):
        self.command.kill()
        
        
 
class ROSIOExecption(Exception):
    pass
        
        
if __name__ == '__main__':
    obj = ROS_Control()
    obj.load_ros_configuration(config="/home/slam4uas/fr0zt_workspace/ros_default.bash")
    print(obj.ROS_PACKAGE_PATH)
    

#    obj.kill_rosmaster()

#    obj.check_rosmaster()
    
    
    
    
    