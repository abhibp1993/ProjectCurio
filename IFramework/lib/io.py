# -*- coding: utf-8 -*-
"""
Created on Mon Sep 21 23:52:56 2015

@author: AbhishekKulkarni
"""

import util
import numpy as np

# Common Configurable Variables
wheelBase = 0.15    #in meter
wheelRadius = 0.07  #in meter

#ULTRASONIC_RANGE_BOUND = 200    #in cm     --> Should come from some config file.


class State(object):
    """
    Represents all internal and external observable state variables for Curio.
    """
    odom    = None
    vel     = None
    acc     = None
    jerk    = None
    sonar   = None
    irprox  = None
    pitch   = None
    roll    = None
    battVolt    = None
    motorFail   = [False, False]        # [motor1, motor2]
    firmwareCommOK  = [True, True]      # [Arduino1, Arduino2]
    baseStation     = 0                 # number of base stations connected

    def rosmessagize(self):
        raise NotImplementedError('State: ROS Message not yet decided.')


class SensorInput(object):
    """
    Represents a set of sensor readings. 
    The class will hold partially pre-preocessed data.
    
    Variables:
        - odom: util.Pose instance: (x, y, theta)
        - vel: 3-list: (v_x, v_y, omega)
        - acc: 3-list: (a_x, a_y, alpha)
        - jerk: 3-list: (j_x, j_y, zeta)
        - sonar: values of each sonar in meters (list of size 5)
        - irprox: values of each ir-proximity sensor (list of size 8)
        - extension: yet to be decided.
        
    """           
    
    def __init__(self, pose = util.Pose(), vel = list(), acc = list(), jerk = list(), sonar = list(), irprox = list()):
        self._odom = pose
        self._vel = vel
        self._acc = acc
        self._jerk = jerk
        self._sonar = sonar
        self._irprox = irprox
    
    @property
    def odom(self):
        return self._odom
    
    @property
    def position(self):
        return np.array([self.odom.x, self.odom.y])
    
    @property
    def vel(self):
        return self._vel
    
    @property
    def acc(self):
        return self._acc
        
    @property
    def jerk(self):
        return self._jerk
        
    @property
    def sonar(self):
        return self._sonar
        
    @property
    def irprox(self):
        return self._irprox

    def rosmessagize(self):
        raise NotImplementedError('ROS Messages not yet finalized here @ io.SensorInput.')
    
    
    
class Action(object):
    """
    An abstraction layer to interface higher level control commands to firmware level
    control algorithms.
    """
#    MINRPM = 25
    
    def __init__(self, fvel = 0.0, rvel = 0.0):
        """
        Constructor.
        
        @fvel: forward velocity (in m/s)
        @rvel: angular velocity of robot w.r.t. it's center (in rad/s)
        """
        self.fvel = fvel       
        self.rvel = rvel  


    def transformToLowLevelCmd(self):
        """
        Converts the high level (v, w) to left and right motor angular velocities, 
        in rpm. 
        
        @returns: list of left and right rpms respectively.
        """
        self.wr = (2 * self.fvel * 60 + self.rvel * wheelBase)/(2 * wheelRadius)  
        self.wl = (2 * self.fvel * 60 - self.rvel * wheelBase)/(2 * wheelRadius)  
        
#        # Protection: For testing
#        if self.wr < self.MINRPM or self.wl < self.MINRPM:
#            self.wl = 0
#            self.wr = 0
#            print 'Warning: Too slow to run.'
            
        return [round(self.wl, 4), round(self.wr, 4)]
        
        
    def rosmessagize(self):
        """
        Generates the action data in frame format as required by ROS. 
        """
        # make use of wr, wl. 
        # if they are not defined, raise error/call the transformToLowLevelCmd method
        raise NotImplementedError('ROS Messages not yet finalized here @ io.Action.')
        
    def __str__(self):
        return str('v = ' + str(self.fvel) + ' m/s' + ', omega = ' + str(self.rvel) + ' rad/s')


    print "imported lib.io succesfully..."
