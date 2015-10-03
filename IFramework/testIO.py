# -*- coding: utf-8 -*-
"""
Created on Mon Sep 21 23:41:51 2015

@author: AbhishekKulkarni
"""

import lib.util as util
import lib.io as io


def testPose():    
    p = util.Pose(10, 90, 270)
    print 'Pose: ', p
    print 'theta: ', p.theta
    print 'theta 0-2pi: ', p.thetaIn0To2Pi
    print 'theta -pi-pi: ', p.thetaInMinusPiToPi
    print 'theta degree: ', p.thetaInDegrees


def testAction():
    #import random
    a = io.Action(0.00, 1.00)
    print 'Action'
    print a
    ret = a.transformToLowLevelCmd()
    print str('omega_L = ' + str(ret[0]) + ' rad/s' + ', omega_R = ' + str(ret[1]) + ' rad/s') 
    

if __name__ == '__main__':
    testAction()