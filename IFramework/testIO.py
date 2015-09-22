# -*- coding: utf-8 -*-
"""
Created on Mon Sep 21 23:41:51 2015

@author: AbhishekKulkarni
"""

import lib.util as util
import lib.io as io

a = io.SensorInput()
p = util.Pose(10, 90, 270)
print 'Pose: ', p
print 'theta: ', p.theta
print 'theta 0-2pi: ', p.thetaIn0To2Pi
print 'theta -pi-pi: ', p.thetaInMinusPiToPi
print 'theta degree: ', p.thetaInDegrees
