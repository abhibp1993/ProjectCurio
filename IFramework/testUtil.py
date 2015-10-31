# -*- coding: utf-8 -*-
"""
Created on Mon Oct  5 13:04:26 2015

@author: abhibp1993
"""

import math
import lib.util as util


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
    

def test_Pose(poseSet):
    """
    @poseSet: list of util.Pose instances
    """
    testCounter = 0
    
    
    print '------------------------------------'
    print 'Properties testing...'
    testCounter = 0
    for p in poseSet:
        testCounter += 1
        print 'Test %d'%testCounter
        print p
        print '\tx, y, t: ', p.x, p.y, p.theta
        print '\tpoint: ', p.point
        print '\tList: ', p.toList()
        print '\tTuple: ', p.toTuple()
        print '\tMatrix: ', p.transform.transformMatrix

    print '------------------------------------'
    print 'Distance, isNear...'
    testCounter = 0
    for i in range(len(poseSet)):
        testCounter += 1
        print 'Test %d'%testCounter
        print poseSet[i], poseSet[(i+1)%len(poseSet)]
        print '\tDist: ', poseSet[i].distance(poseSet[(i+1)%len(poseSet)])
        print '\tNear: ', poseSet[i].isNear(poseSet[(i+1)%len(poseSet)])
        print '\tAdd: ', poseSet[i] + poseSet[(i+1)%len(poseSet)]
        print '\tSub: ', poseSet[i] - poseSet[(i+1)%len(poseSet)]


    print '------------------------------------'
    print 'Transformation...'
    
    T1 = util.Transform(translate=util.Vector2D(p1=util.ORIGIN, p2=util.Point2D(1, 1)))
    T2 = util.Transform(translate=util.Vector2D(p1=util.ORIGIN, p2=util.Point2D(-1, 1)))
    print 'Applying T1: ', T1    
    print 'Applying T2: ', T2
    
    testCounter = 0
    for p in poseSet:
        testCounter += 1
        print 'Test %d'%testCounter
        print 'old: ', p
        p.applyTransform(T1)
        print '\tnew 1: ', p
        p.applyTransform(T2)
        print '\tnew 2: ', p
    
    
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
    
def run_Pose(): 
    p1 = util.Pose(0.0, 0.0, 0.0)
    p2 = util.Pose(1.0, 3.0, 0.0)
    p3 = util.Pose(1.0, -1.0, math.radians(45.0))
    p4 = util.Pose(1.0, 4.0, math.radians(45.1))
    p5 = util.Pose(9e-04, 1e-04, math.radians(45.1))
    p6 = util.Pose(0.0, 0, math.radians(45.09))

    test_Pose([p1, p2, p3, p4, p5, p6])
    

def run_TransformPoint():
    p1 = util.Point2D(x=0, y=0)
    p2 = util.Point2D(x=1, y=1)
    p3 = util.Point2D(x=2, y=3)

    v12 = util.Vector2D(p1, p2)
    v13 = util.Vector2D(p1, p3)
    v23 = util.Vector2D(p2, p3)
    
    T1 = util.Transform(translate=util.Vector2D(r=2, theta=0))
    T2 = util.Transform(translate=util.Vector2D(r=2, theta=math.radians(180)))
    T3 = util.Transform(translate=util.Vector2D(r=2, theta=math.radians(90)))
    
    R1 = util.Transform(rotate=math.radians(45))
    R2 = util.Transform(rotate=math.radians(90))
    R3 = util.Transform(rotate=math.radians(-90))
    
    RT1 = util.Transform(rotate=math.radians(45), translate=util.Vector2D(r=2, theta=math.radians(45)))
    TR1 = util.Transform(rotate=math.radians(45), translate=util.Vector2D(r=2, theta=math.radians(45)), applyRotThenTrans=False)
    
    
    print '---------------------------------'  
    print 'Translate-----'
    print 'T: ', T1, 'P: ', p1
    print 'Output: ', T1 * p1
    print 'T: ', T2, 'P: ', p1
    print 'Output: ', T2 * p1
    print 'T: ', T3, 'P: ', p1
    print 'Output: ', T3 * p1       
    
    print 'Rotate-----'
    print 'T: ', R1, 'P: ', p1
    print 'Output: ', R1 * p1
    print 'T: ', R2, 'P: ', p1
    print 'Output: ', R2 * p1
    print 'T: ', R3, 'P: ', p1
    print 'Output: ', R3 * p1

    print 'Combined-----'
    print 'T: ', RT1, 'P: ', p1
    print 'Output: ', RT1 * p1
    print 'T: ', TR1, 'P: ', p1
    print 'Output: ', TR1 * p1
    
    print 
    
    print '---------------------------------'    
    print 'Translate-----'
    print 'T: ', T1, 'P: ', p2
    print 'Output: ', T1 * p2
    print 'T: ', T2, 'P: ', p2
    print 'Output: ', T2 * p2
    print 'T: ', T3, 'P: ', p2
    print 'Output: ', T3 * p2

    print 'Rotate-----'
    print 'T: ', R1, 'P: ', p2
    print 'Output: ', R1 * p2
    print 'T: ', R2, 'P: ', p2
    print 'Output: ', R2 * p2
    print 'T: ', R3, 'P: ', p2
    print 'Output: ', R3 * p2

    print 'Combined-----'
    print 'T: ', RT1, 'P: ', p2
    print 'Output: ', RT1 * p2
    print 'T: ', TR1, 'P: ', p2
    print 'Output: ', TR1 * p2

    print 
     
    print '---------------------------------'    
    print 'Translate-----'
    print 'T: ', T1, 'P: ', p3
    print 'Output: ', T1 * p3
    print 'T: ', T2, 'P: ', p3
    print 'Output: ', T2 * p3
    print 'T: ', T3, 'P: ', p3
    print 'Output: ', T3 * p3
    
    print 'Rotate-----'
    print 'T: ', R1, 'P: ', p3
    print 'Output: ', R1 * p3
    print 'T: ', R2, 'P: ', p3
    print 'Output: ', R2 * p3
    print 'T: ', R3, 'P: ', p3
    print 'Output: ', R3 * p3
 
    print 'Combined-----'
    print 'T: ', RT1, 'P: ', p3
    print 'Output: ', RT1 * p3
    print 'T: ', TR1, 'P: ', p3
    print 'Output: ', TR1 * p3
   
    print    
    
    print '---------------------------------'  
    print 'Translate-----'
    print 'T: ', T1, 'V: ', v12
    print 'Output: ', T1 * v12
    print 'T: ', T2, 'V: ', v12
    print 'Output: ', T2 * v12
    print 'T: ', T3, 'V: ', v12
    print 'Output: ', T3 * v12       
    
    print 'Rotate-----'
    print 'T: ', R1, 'V: ', v12
    print 'Output: ', R1 * v12
    print 'T: ', R2, 'V: ', v12
    print 'Output: ', R2 * v12
    print 'T: ', R3, 'V: ', v12
    print 'Output: ', R3 * v12

    print 'Combined-----'
    print 'T: ', RT1, 'V: ', v12
    print 'Output: ', RT1 * v12
    print 'T: ', TR1, 'V: ', v12
    print 'Output: ', TR1 * v12
    
    print 
    
    print '---------------------------------'    
    print 'Translate-----'
    print 'T: ', T1, 'V: ', v13
    print 'Output: ', T1 * v13
    print 'T: ', T2, 'V: ', v13
    print 'Output: ', T2 * v13
    print 'T: ', T3, 'V: ', v13
    print 'Output: ', T3 * v13      
    
    print 'Rotate-----'
    print 'T: ', R1, 'V: ', v13
    print 'Output: ', R1 * v13
    print 'T: ', R2, 'V: ', v13
    print 'Output: ', R2 * v13
    print 'T: ', R3, 'V: ', v13
    print 'Output: ', R3 * v13

    print 'Combined-----'
    print 'T: ', RT1, 'V: ', v13
    print 'Output: ', RT1 * v13
    print 'T: ', TR1, 'V: ', v13
    print 'Output: ', TR1 * v13
    
    print 
     
    print '---------------------------------'    
    print 'Translate-----'
    print 'T: ', T1, 'V: ', v23
    print 'Output: ', (T1 * v23).p1, (T1 * v23).mag, math.degrees((T1 * v23).arg)
    print 'T: ', T2, 'V: ', v23
    print 'Output: ', (T2 * v23).p1, (T2 * v23).mag, math.degrees((T2 * v23).arg)
    print 'T: ', T3, 'V: ', v23
    print 'Output: ', (T3 * v23).p1, (T3 * v23).mag, math.degrees((T3 * v23).arg)    
    
    print 'Rotate-----'
    print 'T: ', R1, 'V: ', v23
    print 'Output: ', R1 * v23
    print 'T: ', R2, 'V: ', v23
    print 'Output: ', R2 * v23
    print 'T: ', R3, 'V: ', v23
    print 'Output: ', R3 * v23

    print 'Combined-----'
    print 'T: ', RT1, 'V: ', v23
    print 'Output: ', (RT1 * v23).p1, (RT1 * v23).mag, math.degrees((RT1 * v23).arg), (RT1 * v23).p2
    print 'T: ', TR1, 'V: ', v23
    print 'Output: ', (TR1 * v23).p1, (TR1 * v23).mag, math.degrees((TR1 * v23).arg), (TR1 * v23).p2
    
    print 


def run_Circle():
    import matplotlib.pyplot as plt
    c1 = util.Circle()
    P = util.Point2D(x=3, y=4)
    
    tang = c1.tangent(P)
    vecPC  = util.Vector2D(p1=P, p2=c1.C)
    vecPT1 = tang[0]
    vecPT2 = tang[1]
    
    xPC = [vecPC.p1.x, vecPC.p2.x]
    yPC = [vecPC.p1.y, vecPC.p2.y]
    xPT1 = [vecPT1.p1.x, vecPT1.p2.x]
    yPT1 = [vecPT1.p1.y, vecPT1.p2.y]
    xPT2 = [vecPT2.p1.x, vecPT2.p2.x]
    yPT2 = [vecPT2.p1.y, vecPT2.p2.y]

    pltCircle1 = plt.Circle(c1.C.toList(), c1.r, color='r')
    
    fig = plt.gcf()
    ax = plt.gca()
    ax.cla() 
    ax.axis("equal")
    ax.set_xlim((-5,5))

    ax.plot(xPC , yPC , 'b-')
    ax.plot(xPT1, yPT1, 'b-')
    ax.plot(xPT2, yPT2, 'b-')
    fig.gca().add_artist(pltCircle1)
    
    #fig.savefig('plotcircles.png')
    
    
    
if __name__ == '__main__':
    #run_Point2D()
    #run_Vector2D()
    #run_Pose()
    #run_TransformPoint()
    run_Circle()