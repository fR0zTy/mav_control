#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Tue Oct  4 16:36:51 2016

@author: fr0zty
"""
import rospkg
import numpy as np

def get_twist(twist_velocity,key_events, ):
    
    dx, dy, dz = twist_velocity
    dax, day, daz = twist_velocity
    
    key_events = key_events.key()
    
    
    twist_events = {  #x, y, z, pitch, roll, yaw
                    controls[0]: [dx, 0, 0, 0, 0, 0],
                    controls[1]: [-dx, 0, 0, 0, 0, 0],
                    QtCore.Qt.Key_A: [0, dy, 0, 0, 0, 0],
                    controls[3]: [0, -dy, 0, 0, 0, 0], 
                    controls[4]: [0, 0, dz, 0, 0, 0],
                    controls[5]: [0, 0, -dz, 0, 0, 0],
                    controls[6]: [0, 0, 0, 0, 0, daz],
                    controls[7]: [0, 0, 0, 0, 0, -daz],
                    }
    return twist_events[key_events]



