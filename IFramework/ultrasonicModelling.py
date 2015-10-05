# -*- coding: utf-8 -*-
"""
Created on Sat Oct  3 23:35:11 2015

@author: abhibp1993
"""

import lib.util as util
import pylab
import math

class Ultrasonic(object):
    def __init__(self, leftPoint, absAngle = 0.0, length = 0.05, maxRange = 2.0):
        """
        Constructor.
        
        Remark: Refer to diary for detailed calculations.
        """
        self.angle = absAngle        
        self.range = maxRange
        print 'Polling Time: %f ms'%(40.0*self.range/4.0)
        
        O = util.Point2D()
        OA = util.DirectedLineSeg(O, O + util.Point2D(length, 0)).rotate(self.angle)
        OM = 0.5 * OA
        L = self.range * math.acos(15 * math.pi / 180.0)
        MLeft  = util.Point2D(L * math.cos(self.angle + (math.pi / 8)), \
                                   L * math.sin(self.angle + (math.pi / 8)))
        MRight = util.Point2D(L * math.cos(self.angle - (math.pi / 8)), \
                                   L * math.sin(self.angle - (math.pi / 8)))
        
        # Actual Points
        self.O = leftPoint
        self.A = OA.p2 + leftPoint
        self.M = OM.p2 + leftPoint
        self.MLeft = MLeft + leftPoint
        self.MRight = MRight + leftPoint
        
        
    @property
    def region(self):
        return [self.M, self.MLeft, self.MRight]
    
    
if __name__ == '__main__':
    
    # Ultrasonic 1
    u1 = Ultrasonic(leftPoint=util.Point2D(0, 0.25), absAngle=math.pi/2, maxRange=1.5)
    u2 = Ultrasonic(leftPoint=util.Point2D(-0.035, 0.23), absAngle=( 40.0/180*math.pi + math.pi/2), maxRange=1.5)
    u3 = Ultrasonic(leftPoint=util.Point2D( 0.035, 0.23), absAngle=(-40.0/180*math.pi + math.pi/2), maxRange=1.5)
    u4 = Ultrasonic(leftPoint=util.Point2D(-0.07, 0.076), absAngle=( 80.0/180*math.pi + math.pi/2), maxRange=1.6)
    u5 = Ultrasonic(leftPoint=util.Point2D( 0.07, 0.076), absAngle=(-80.0/180*math.pi + math.pi/2), maxRange=1.6)
    
    reg1 = u1.region
    reg2 = u2.region
    reg3 = u3.region
    reg4 = u4.region
    reg5 = u5.region
    
    x1 = [p.x for p in reg1]
    y1 = [p.y for p in reg1]
    x1.append(x1[0])
    y1.append(y1[0])
    
    x2 = [p.x for p in reg2]
    y2 = [p.y for p in reg2]
    x2.append(x2[0])
    y2.append(y2[0])

    x3 = [p.x for p in reg3]
    y3 = [p.y for p in reg3]
    x3.append(x3[0])
    y3.append(y3[0])

    x4 = [p.x for p in reg4]
    y4 = [p.y for p in reg4]
    x4.append(x4[0])
    y4.append(y4[0])
    
    x5 = [p.x for p in reg5]
    y5 = [p.y for p in reg5]
    x5.append(x5[0])
    y5.append(y5[0])

    pylab.figure(1)
    pylab.xlim([-2.5, 3])
    pylab.ylim([-1.0, 4])
    pylab.plot(x1, y1, 'b-')
    pylab.plot(x2, y2, 'b-')
    pylab.plot(x3, y3, 'b-')
    pylab.plot(x4, y4, 'b-')
    pylab.plot(x5, y5, 'b-')
    
    pylab.show()