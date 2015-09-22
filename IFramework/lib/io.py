# -*- coding: utf-8 -*-
"""
Created on Mon Sep 21 23:52:56 2015

@author: AbhishekKulkarni
"""


print "Importing io"

import util
import numpy as np

# Common Configurable Variables
#ULTRASONIC_RANGE_BOUND = 200    #in cm     --> Should come from some config file.



class SensorInput(object):
    """
    Represents a set of sensor readings. 
    The class will hold partially pre-preocessed data.
    
    Variables:
        - pose: Current state pose (instance of IFramework.lib.util.Pose class)
        - sonar: values of each sonar in meters (list of size 5)
        - irprox: values of each ir-proximity sensor (list of size 8)
        - imu: latest reading of RPY angles, accel XYZ. (instance of IFramework.lib.io.SensorInput.IMU class)
        - state: Complete state of Robot. (Instance of IFramework.lib.io.SensorInput.CurioState class)
        
    """
    class IMU(object):
        def __init__(self, rpy = list(), accel = list()):
            self.rpy = rpy
            self.accel = accel
    
    
    class State(object):
        def __init__(self):
            self.x = 0.0
            self.velx = 0.0
            self.accx = 0.0
            self.jerx = 0.0
            self.y = 0.0
            self.vely = 0.0
            self.accy = 0.0
            self.jery = 0.0
            self.theta = 0.0
            self.omega = 0.0
            self.alpha = 0.0
            self.jert = 0.0
            self.pitch = 0.0
            self.roll = 0.0
            self.timestamp = 0
            
    

    def __init__(self, pose = util.Pose(), sonar = list(), irprox = list(), imu = IMU(), state = State()):
        self.pose = pose
        self.sonar = sonar
        self.irprox = irprox
        self.imu = imu
        self.state = state
    
    @property
    def odometry(self):
        return self.pose
    
    @property
    def rpy(self):
        return self.imu.rpy
        
    @property
    def position(self):
        return np.array([self.state.x, self.state.y])
    
    @property
    def velocity(self):
        return np.array([self.state.velx, self.state.vely])
        
    @property
    def acceleration(self):
        return np.array([self.state.accx, self.state.accy])
        
    @property
    def jerk(self):
        return np.array([self.state.jerx, self.state.jery])

        
    
    
    
    
class Action(object):
    pass
