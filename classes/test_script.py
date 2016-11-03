#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@author: fr0zty
"""

import sys

try:
    import rospy
    from mav_control.nodes import SubscriberNode
except ImportError as ie:
    print('Import package missing',ie)
    

