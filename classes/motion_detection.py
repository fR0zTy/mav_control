#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Sat Oct 15 15:31:38 2016

@author: fr0zty
"""

try:
    import cv2
    import datetime
    #import imutils
except ImportError as ie:
    print('Imports not found',ie)
    
#class motion_detection(object):
#    
#    def __init__(self):
#        self.webcam_status = None
#        self.area_size = 0
#        
#        if self.

cam = cv2.VideoCapture(0)