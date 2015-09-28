# -*- coding: utf-8 -*-
"""
Created on Mon Sep 21 23:52:56 2015

@author: AbhishekKulkarni
"""
print "Importing util"

import math
import numpy as np

class Pose(object):
    def __init__(self, x = None, y = None, theta = None):
        self.x = x
        self.y = y
        self._theta = np.deg2rad(theta)
     
    @property 
    def thetaIn0To2Pi(self):
        return (self._theta) % (2 * math.pi)
    
    @property
    def thetaInMinusPiToPi(self):
        angle = self.thetaIn0To2Pi
        if angle > math.pi:
            angle -= 2*math.pi
        
        return angle
    
    @property
    def thetaInDegrees(self):
        return math.degrees(self._theta) % 360
        
    @property 
    def theta(self):
        return self._theta
        
    @theta.setter
    def theta(self, val):
        """
        Sets the heading angle (in radians) in range (-inf, +inf)
        """
        self._theta = val
    
    def __str__(self):
        return str([self.x, self.y, self.thetaInDegrees])
    