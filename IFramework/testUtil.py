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
    


def test_Vector2D(vSet):
    """
    @pTupleSet: list of 2-tuples of (p1, p2)
    @rthetaTupleSet: list of 3-tuples of (p1, r, theta)
    """
    testCounter = 0
    
    # Test: Instantiation
    import random
    
    v = util.Vector2D(util.ORIGIN, util.ORIGIN)       #Zero Vector instantiation
    for i in range(100):
        p1 = util.Point2D(x=random.random(), y=random.random())
        p2 = util.Point2D(x=random.random(), y=random.random())
    
        try: 
            v = util.Vector2D(p1, p2)
            testCounter += 1
        except:
            print 'Test %d'%testCounter
            print p1, p2, 'Vector instantiation failed.'
        
    print 'Instantiation Tests Passed'
        
    
    print '------------------------------------'
    print 'Properties testing...'
    testCounter = 0
    for v in vSet:
        testCounter += 1
        print 'Test %d'%testCounter
        print v
        print 'x: ', v.x
        print 'y: ', v.y
        print 'unitVector: ', v.unitVector
        print 'length:', v.length
        print ''
    
    
    print '------------------------------------'
    print 'Normalize...'
    testCounter = 0
    temp = [t for t in vSet]    #copy to avoid mutation
    for v in temp:
        testCounter += 1
        print 'Test %d'%testCounter
        print 'old: ', v
        v.normalize()
        print 'new: ', v
        print 'length:', v.length
        print ''                

    print '------------------------------------'
    print 'CCW, CW, Collinear, in...'
    testCounter = 0
    temp = [t for t in vSet]    #copy to avoid mutation
    for v in temp:
        pSet = [v.p1, v.p2, v.p1 + util.Point2D(x=0.01), v.p1 + util.Point2D(y=-0.009), \
                v.p1 + util.Point2D(x=1.0, y=1.0), v.p1 + util.Point2D(x=-1.0, y=1.0), \
                v.p1 + util.Point2D(x=-1.0, y=-1.0), v.p1 + util.Point2D(x=1.0, y=-1.0)]
        testCounter += 1
        print 'Test %d'%testCounter
        print v
        for p in pSet:
            print '\t', p, v.isccw(p), v.iscw(p), v.collinear(p), p in v
    
    print '------------------------------------'
    print 'Add Subtract Dot Cross...'
    testCounter = 0
    temp = [t for t in vSet]    #copy to avoid mutation
    for i in range(len(temp)):
        testCounter += 1
        print 'Test %d'%testCounter
        print 'Vector', temp[i], ',', temp[(i+1)%len(temp)]
        print 'Add', temp[i] + temp[(i+1)%len(temp)]
        print 'Sub', temp[i] - temp[(i+1)%len(temp)]
        print 'Dot', temp[i] * temp[(i+1)%len(temp)]
        print 'Crs', temp[i] ** temp[(i+1)%len(temp)]
        print ''
        
   
    print '------------------------------------'
    print 'Intersections...'
    
    p1 = util.Point2D(10, 0)
    p2 = util.Point2D(0, 10)
    p3 = util.Point2D(0, 0)
    p4 = util.Point2D(10, 10)
    v1 = util.Vector2D(p1, p2)
    v2 = util.Vector2D(p3, p4)
    print p1, p2, p3, p4, v1.intersect(v2)    
    
    p1 = util.Point2D(-5, 5)
    p2 = util.Point2D(0, 0)
    p3 = util.Point2D(1, 1)
    p4 = util.Point2D(10, 10)
    v1 = util.Vector2D(p1, p2)
    v2 = util.Vector2D(p3, p4)
    print p1, p2, p3, p4, v1.intersect(v2)    
    
    
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
    
    
def run_Vector2D(): 
    v1 = util.zeroVector
    v2 = util.Vector2D(util.ORIGIN, util.Point2D(1, 0))     
    v3 = util.Vector2D(util.ORIGIN, util.Point2D(0, 1))     
    v4 = util.Vector2D(util.ORIGIN, util.Point2D(4, 0))     
    v5 = util.Vector2D(util.ORIGIN, util.Point2D(0, 5))     
    v6 = util.Vector2D(util.ORIGIN, util.Point2D(-1, 0))     
    v7 = util.Vector2D(util.ORIGIN, util.Point2D(0, -1))     
    v8 = util.Vector2D(util.ORIGIN, util.Point2D(1, 1))     
    v9 = util.Vector2D(util.ORIGIN, util.Point2D(-1, 1))     
    v10 = util.Vector2D(util.ORIGIN, util.Point2D(-1, -1))     
    v11 = util.Vector2D(util.ORIGIN, util.Point2D(1, -1))     
    
    test_Vector2D([v1, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11])
    
if __name__ == '__main__':
    #run_Point2D()
    run_Vector2D()