#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
@author: fr0zty
"""

# Module : utilities
import sys
import os

try:
    import json
    import rospkg
    import rospy
    import warnings
    from std_msgs.msg import String
except ImportError as ie:
    print("Import modules missing", ie)
    
    


def create_configuration_file(mode="default", pkg_name=""):
    '''
    Creates a software configuration file with parameteres for MAV_Control GUI
    
    Usage:
        >>>CreateConfigurationFile(mode) \n
        
            mode = default -> creates mav_control_config.json file with default values \n
            mode = custom -> creates mav_control_config.json file with empty values for user to modify
    
    '''
        
    if pkg_name == "":
        warnings.warn("Please specify package name")
        sys.exit()
        
    rospack = rospkg.RosPack()
    config = {}
    config_path = os.path.join(rospack.get_path(pkg_name), "mav_control_configuration")

    if mode=="default":
        loc = ["sim", "rc", "", "", "", rospack.get_path(pkg_name), "", "", os.path.join(rospack.get_path(pkg_name), "rviz_cfg")]
        val = [4, 2, 20, 1]

    elif mode == "custom":
        loc = ["", "", "", "", "", rospack.get_path(pkg_name), "", "", ""]
        val = [0, 0, 0, 0]
               
               
    config["operation_mode"] = loc[0]
    config["controller_status"] = loc[1]
    
    config["laser_scan_topic"] = loc[2]
    config["stereo_cam_right_topic"] = loc[3]
    config["stereo_cam_left_topic"] = loc[4]
    
    config["output_directory"] =  loc[5]
    config["ros_configuration"] = loc[6]
    config["gazebo_configuration"] = loc[7]
    config["rviz_configuration"] = loc[8]

    config["warning_distance"] = val[0]
    config["auto_distance"] = val[1]
    config["rate"] = val[2]
    config["twist_velocity"] = val[3]


    if not os.path.exists(config_path):
        os.mkdir(os.path.join(config_path))
        
    with open(os.path.join(config_path,'mav_control_config.json'), mode="w") as f:
            json.dump(config, f)
            
    f.close()
                