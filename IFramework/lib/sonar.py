# -*- coding: utf-8 -*-
"""
Created on Tue Oct  6 13:27:19 2015

@author: abhibp1993
"""

import util
import math


class Sonar(object):
    _MAXRANGE = 4.0
    _SENSEANGLE = math.radians(22.5)
    
    def __init__(self, pose, scanRange = 2.0):
        assert isinstance(pose, util.Pose), 'pose must be a util.Pose instance'
        assert isinstance(scanRange, (float, int)), 'pose must be a float or integer'
        assert scanRange < self._MAXRANGE, 'scan range cannot be greater than Max Range'
        
        self.pose = pose            # pose in global coordinates
        self.scanRange = scanRange  # operating scan range of ultrasonic
        
    @property
    def region(self):
        """ Returns polygon (triangle) covering the region sensed by sonar sensor """        
    
        rotLeft  = util.Transform(rotate=math.radians(-22.5))
        rotRight = util.Transform(rotate=math.radians( 22.5))
        translate = util.Transform(translate=util.Vector2D(util.ORIGIN, util.Point2D(self.scanRange, 0)))
        
        p1 = self.pose.point
        p2 = (translate * (rotLeft  * self.pose)).point
        p3 = (translate * (rotRight * self.pose)).point

        return util.Polygon([p1, p2, p3])
        
        
    
    def _draw(self, fig):
        reg = self.region.pSet
        fig
        x = [p.x for p in reg]
        y = [p.y for p in reg]
        x.append(x[0])
        y.append(y[0])        
        pylab.plot(x, y, 'b-')
        

if __name__ == '__main__':
    import pylab
    
    sonar1 = Sonar(util.Pose( 0.000, 0.25, math.radians(90)),  scanRange=1.5)
    sonar2 = Sonar(util.Pose(-0.035, 0.23, math.radians(130)), scanRange=1.5)
    sonar3 = Sonar(util.Pose( 0.035, 0.23, math.radians(50)),  scanRange=1.5)
    sonar4 = Sonar(util.Pose(-0.07, 0.076, math.radians(170)), scanRange=1.6)
    sonar5 = Sonar(util.Pose( 0.07, 0.076, math.radians(10)),  scanRange=1.6)
    
    fig = pylab.figure(1)
    pylab.xlim([-5,5])
    pylab.ylim([-5,5])
    sonar1._draw(fig)
    sonar2._draw(fig)
    sonar3._draw(fig)
    sonar4._draw(fig)
    sonar5._draw(fig)
    pylab.show()