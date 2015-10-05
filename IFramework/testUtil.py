# -*- coding: utf-8 -*-
"""
Created on Mon Oct  5 13:04:26 2015

@author: abhibp1993
"""

import math
import lib.util1 as util


def test_Point2D(pSet):
    testCounter = 0
    
    # Test: x, y, positionVector Properties
    print '------------------------------------'
    print 'Properties testing...'
    testCounter = 0
    for p in pSet:
        testCounter += 1
        print 'Test %d'%testCounter
        print p
        print 'x: ', p.x
        print 'y: ', p.y
        print 'positionVector: ', p.positionVector
        print 'list:', p.toList()
        print ''
        
    print '------------------------------------'
    print 'Distance...'
    testCounter = 0
    for i in range(len(pSet)):        
        testCounter += 1
        print 'Test %d'%testCounter
        print pSet[i], pSet[(i+1)%len(pSet)], pSet[i].distTo(pSet[(i+1)%len(pSet)])
        print ''
        
    print '------------------------------------'
    print 'Angles...'
    testCounter = 0
    for i in range(len(pSet)):     
        testCounter += 1
        print 'Test %d'%testCounter
        print pSet[i], pSet[(i+1)%len(pSet)], math.degrees(pSet[i].angleTo(pSet[(i+1)%len(pSet)]))
        print ''
    
    print '------------------------------------'
    print 'Near...'
    testCounter = 0    
    for i in range(len(pSet)):    
        testCounter += 1
        print 'Test %d'%testCounter
        print pSet[i], pSet[(i+1)%len(pSet)], pSet[i].isNear(pSet[(i+1)%len(pSet)])
        print ''

    print '------------------------------------'    
    print 'Equal...'
    testCounter = 0
    for i in range(len(pSet)):        
        testCounter += 1
        print 'Test %d'%testCounter
        print pSet[i], pSet[(i+1)%len(pSet)], pSet[i] == pSet[(i+1)%len(pSet)]
        print ''

    print '------------------------------------'    
    print 'Add...'
    testCounter = 0
    for i in range(len(pSet)):        
        testCounter += 1
        print 'Test %d'%testCounter
        print pSet[i], pSet[(i+1)%len(pSet)], pSet[i] + pSet[(i+1)%len(pSet)]
        print ''

    print '------------------------------------'        
    print 'Subtract...'
    testCounter = 0
    for i in range(len(pSet)):        
        testCounter += 1
        print 'Test %d'%testCounter
        print pSet[i], pSet[(i+1)%len(pSet)], pSet[i] - pSet[(i+1)%len(pSet)]
        print ''
    
    
def run_Point2D():
    """
    Run this function and observe the output. 
    The tests are all successful on 5 Oct 2015, 1400 (abhibp1993)
    
    Remark: The points are chosen in a way that all next point is in next quadrant
        w.r.t. current point. Hence, each angle is 0, 45, 90, ... 
        Some points are chosen equal (p1 = p2), while some are chosen close. 
    """
    O       = util.ORIGIN
    p1      = util.Point2D(1.0, 0.0)
    p2      = util.Point2D(1.0, 0.0)
    p3      = util.Point2D(2.0, 2.0)
    p4      = util.Point2D(2.0, 3.0)
    p5      = util.Point2D(1.0, 4.0)
    p6      = util.Point2D(0.0, 4.0)
    p7      = util.Point2D(-1.0, 3.0)
    p8      = util.Point2D(-1.0, 2.0)
    p9      = util.Point2D(0.0, 1.0)
    p10     = util.Point2D(0.0, 0.0099)
    
    
    # Following test checks properties, distance, angles in all aspects.
    test_Point2D([O, p1, p2, p3, p4, p5, p6, p7, p8, p9, p10])    
    
    
    
    
if __name__ == '__main__':
    run_Point2D()