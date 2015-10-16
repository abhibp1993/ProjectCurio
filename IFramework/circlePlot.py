# -*- coding: utf-8 -*-
"""
Created on Thu Oct 15 18:34:01 2015

@author: AbhishekKulkarni
"""

import matplotlib.pyplot as plt
import lib.util as util
import math


C = util.Point2D(x=1, y=1)
P = util.Point2D(x=3, y=4)
r = 1

vecPC = util.Vector2D(p1=P, p2=C)
angle = math.asin(r/P.distTo(C))

T = util.Transform(rotate=-angle)
vecPT = T * vecPC
T.rotate = angle
vecPT2 = T * vecPC
print 'Transformed', vecPT

vecPT.normalize()
vecPT = vecPT * (P.distTo(C) * math.cos(angle))

xPC = [vecPC.p1.x, vecPC.p2.x]
yPC = [vecPC.p1.y, vecPC.p2.y]
xPT = [vecPT.p1.x, vecPT.p2.x]
yPT = [vecPT.p1.y, vecPT.p2.y]
xPT2 = [vecPT2.p1.x, vecPT2.p2.x]
yPT2 = [vecPT2.p1.y, vecPT2.p2.y]

print vecPC, vecPT.length

#C = (0., 0.)
#R = 1.
#P = [4., 3.]
#
#den = P[0]**2 - P[1]**2
#x = R**4 - R**2 * P[1]**2 / den
#y = R**2 * P[0]**2 - R**4 / den
#contact = [x, y]
#print x, y
#
circle1 = plt.Circle(C.toList(), r, color='r')

ax = plt.gca()
ax.cla() # clear things for fresh plot
ax.axis("equal")
# change default range so that new circles will work
ax.set_xlim((-5,5))
#ax.set_ylim((-5,5))


fig = plt.gcf()
ax.plot(xPC, yPC, 'b-')
ax.plot(xPT, yPT, 'b-')
ax.plot(xPT2, yPT2, 'b-')
fig.gca().add_artist(circle1)
fig.savefig('plotcircles.png')

