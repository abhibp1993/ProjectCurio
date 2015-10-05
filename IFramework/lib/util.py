# -*- coding: utf-8 -*-
"""
Created on Mon Sep 21 23:52:56 2015

@author: AbhishekKulkarni
"""
print "Importing util"

import math
import numpy as np

class Point2D(object):
    def __init__(self, x = 0.0, y = 0.0):
        self.x = x
        self.y = y
        
    def translate(self, obj):
        """
        Translates the current instance by new point vector.
        
        @obj: Point2D instance.
        """
        if isinstance(obj, Point2D):
            return Point2D(self.x + obj.x, self.y + obj.y)
        elif isinstance(obj, DirectedLineSeg):
            p = obj.p2 - obj.p1
            return Point2D(self.x + p.x, self.y + p.y)

    @property            
    def matrix(self):
        return np.matrix([[self.x], [self.y]])
        
    def __sub__(self, point):
        return Point2D(self.x - point.x, self.y - point.y)
    
    def __add__(self, point):
        return Point2D(self.x + point.x, self.y + point.y)
        
    def __str__(self):
        return 'P(' + str(self.x) + ', ' + str(self.y) + ')'
        
        

class DirectedLineSeg(object):    
    def __init__(self, p1 = Point2D(), p2 = Point2D()):
        self.p1 = p1
        self.p2 = p2
    
    def rotate(self, angleInRad):
        C = math.cos(angleInRad)
        S = math.sin(angleInRad)
        T = np.matrix([[C, S], [-S, C]])
        
        newPoint = self.p1.matrix + T * (self.p2 - self.p1).matrix        
        return DirectedLineSeg(self.p1, Point2D(newPoint.tolist()[0][0], newPoint.tolist()[1][0]))
        
    def normalize(self):
        pass
    
    def __mul__(self, obj):
        if isinstance(obj, (int, float)):
            return DirectedLineSeg(Point2D(self.p1.x * obj, self.p1.y * obj), \
                                   Point2D(self.p2.x * obj, self.p2.y * obj))
                    
    def __str__(self):
        return 'DLS(' + str(self.p1) + ', ' + str(self.p2) + ')'
    
    __rmul__ = __mul__   # reverse multiplication (0.5 * dls) is now permitted
        

class Pose(object):
    def __init__(self, x = 0.0, y = 0.0, theta = 0.0):
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
    