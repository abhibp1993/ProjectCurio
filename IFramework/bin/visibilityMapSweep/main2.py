# -*- coding: utf-8 -*-
"""
Created on Sun Oct 25 10:01:30 2015

@author: AbhishekKulkarni
"""

import sys
import os
dirIFramework = os.path.dirname(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))
sys.path.append(dirIFramework)

import random
import math
import copy
import operator

import _animator as anim
import lib.sm as sm
import lib.util as util

util.Polygon.draw = anim._polyDraw

class Path(object):
    def __init__(self, nodeList = []):
        self.points = [n.point for n in nodeList]
    
    @property
    def immediateEdge(self):
        return Path([Node(self.points[0]), Node(self.points[1])])
        
    def __str__(self):
        n = len(self.points)
        string = ''
        for i in range(n-1):
            string += str(self.points[i]) + '->'
        
        string += str(self.points[n-1])
        return string

Path.draw = anim._pathDraw


#######################################################################
## V-GRAPH GENERATION


def rotSweep1(p, pSet, edgeSet):
    
    # Construct set E as suggested by Choset
    tmp = [(p.angleTo(t), t) for t in pSet]    
    tmp.sort(key=operator.itemgetter(0))
    E = [t[1] for t in tmp]
    
    # Construct the sweep line - parallel to X axis
    xmax = [t.x for t in pSet]
    if len(xmax) < 1:
        return []
    xmax = max(xmax) 
    
    l = util.Vector2D(p1=p, p2=util.Point2D(2*xmax, p.y))

    # Active Sorted list of edges intersecting the horizontal (initialization)
    S = []    
    for e in edgeSet:
        if l.intersect(e) == True:
            S.append(e)
    S.sort(key=lambda x: p.distTo(x.getIntersection(l)))
    
    # iterate for checking visibility
    visEdges = []
    for v in E:
        if util.Vector2D(p, v).intersect(S[0]) == False:
            visEdges.append(util.Vector2D(p, v))
        
        for t in edgeSet:
            if t.p1 == v:
                S.append(t)
        
        for t in edgeSet:
            if t.p2 == v and t in S:
                S.remove(t)
        
        S.sort(key=lambda x: p.distTo(x.getIntersection(l)))
    
    return visEdges
    

def rotSweep(p, pSet, edgeSet):
    
    # Construct set E as suggested by Choset
    tmp = [(p.angleTo(t), t) for t in pSet]    
    tmp.sort(key=operator.itemgetter(0))
    E = [t[1] for t in tmp]
    
    # Construct the sweep line - parallel to X axis
    xmax = [t.x for t in pSet]
    if len(xmax) < 1:
        return []
    xmax = max(xmax) 
    
    l = util.Vector2D(p1=p, p2=util.Point2D(2*xmax, p.y))
    
    # Active Sorted list of edges intersecting the horizontal (initialization)
    S = []    
    for e in edgeSet:
        if l.intersect(e) == True:
            S.append(e)
    S.sort(key=lambda x: p.distTo(x.getIntersection(l)))
    
    # iterate for checking visibility
    visEdges = []
    for v in E:
        if util.Vector2D(p, v).intersect(S[0]) == False:
            visEdges.append(util.Vector2D(p, v))
        
        for t in edgeSet:
            if t.p1 == v:
                S.append(t)
        
        for t in edgeSet:
            if t.p2 == v and t in S:
                S.remove(t)
                
    return visEdges
    

def VisibilityGraph(polySet, start, goal):
    G = util.Graph(allowDuplication=False)

    # Extract all points
    points = []    
    points.append(start)
    points.append(goal)
    for poly in polySet:
        points.extend(poly.vertices)
    print 'points extracted...'
    
    # Collection of all edges from all polygons
    polyEdges = []
    for p in polySet:
        polyEdges.extend(p.edges)
    print 'edges extracted...'
    
    # Construct set of visbility edges from each point
    visEdges = []
    n = len(points)
    for i in range(n):
        p = points[i]
        try:
            print '\tIter: %d - rotSweep started...'%i
            tmp = rotSweep(p, points[i+1:], polyEdges)
            print '\tIter: %d - rotSweep ended...'%i
        except Exception, ex:
            print ex.message
            
        visEdges.extend(tmp)
        
    # All polygon edges are edges in visibility graph
    visEdges.extend(polyEdges)
    
    # Construct Graph
    #print 'points being added %d'%len(points)
#    V = []
#    for p in points:
#        V.append(G.addNode(p, str(p)))
        
    #G.E = [Edge(e) for e in visEdges]
    for e in visEdges:
        n1 = G.addNode(e.p1, str(e.p1))
        n2 = G.addNode(e.p2, str(e.p2))
        G.addEdge(n1, n2, 1)  
        
    return G
    

#######################################################################
## STATE MACHINES


class Map(sm.SM):
    def __init__(self, boundary, statObs = [], mvObs = []):
        self.boundary = boundary        # boundary as polygon
        self.statObs  = statObs         # static non-moving obstacles
        self.mvObs    = mvObs           # dynamic or moving obstacles
        
    @property
    def length(self):
        return max([edg.length for edg in self.boundary.edges])
        
        
    def getNextValues(self, state, inp):
        T = util.Transform()
        
        newMvObs = []
        for obs in self.mvObs:
            if random.random() < 0.5:
                T.rotate = math.radians(1 * random.random() - 1)
            else:
                x = (random.random() - 0.05) * 0.01
                y = (random.random() - 0.05) * 0.01
                T.translate = util.Vector2D(util.ORIGIN, util.Point2D(x, y))
            
            newObstacle = T * obs
            newMvObs.append(newObstacle)
        
        self.mvObs = newMvObs        
        return (None, None)
        
    def draw(self, canvas):
        scale = canvas.get_width()/self.length
        for obs in self.statObs:
            obs.draw(canvas, color=(255, 255, 0), scale=scale)
            
        for obs in self.mvObs:
            obs.draw(canvas, color=(255, 0, 0), scale=scale)



class Robot(sm.SM):
    def __init__(self, loc, goal):
        self.state = loc     # initialize the position
        self.goal = goal     # initialize the goal
        
        self.currState = self.state
    
    def initialize(self):
        self.doneCondition = (lambda x: x.distTo(self.goal) < 0.1)
        
    def getNextValues(self, state, inp):
        """
        inp = map
        
        Machine internal function:
            1. based on map, compute visibility graph.
            2. based on visibility graph, get next heading location using A* or Dijkstra.
            3. return output as (next goal location, visibility graph).
        """
        print 'Step 1: Inside Robot Machine'
        
        obs = []
        obs = copy.deepcopy(inp.statObs)
        obs.extend(copy.deepcopy(inp.mvObs))
        
        print 'Step 2: Computing Visibility Graph...'
        g = VisibilityGraph(obs, state, self.goal)
        for n in g.nodeList:
            print '\t', n
        
        print '---------------'
        for e in g.edgeList:
            print '\t', e
            
        print 'Step 3: Computed Visibility Graph...'

        print 'Step 4: Computing Optimum Path...'    
        path = util.AStar(g, g.addNode(state, 'start'), g.addNode(self.goal, 'goal'))
        print 'Step 5: Optimum Path Computed...'
#        path = Path(path)
        

#        path = AStar(g, Node(state), Node(self.goal))

#        print state, self.goal, path
#        path = Path(path)
        
        
#        nextGoal = path.immediateEdge
#        goalPoint = nextGoal.points[-1]

#        tr = util.Vector2D(state, goalPoint)
#        tr = util.Vector2D(p1=state, r=0.05, theta=tr.arg)
#        T = util.Transform(translate=tr)
#        newState = T * state        
        
        return (state, g) #(newState, (g, path, nextGoal))

if __name__ == "__main__":
    
    # Construct world    
    bound = util.Polygon([util.ORIGIN, util.Point2D(0, 10), util.Point2D(10, 10), util.Point2D(10, 0)])
    sObs1 = util.Polygon([util.Point2D(2, 4), util.Point2D(3, 5), util.Point2D(2.5, 7)])
    sObs2 = util.Polygon([util.Point2D(6.5, 4), util.Point2D(8, 4), util.Point2D(8, 8), util.Point2D(6.5, 8)])
    sObs3 = util.Polygon([util.Point2D(5, 0), util.Point2D(5, 5), util.Point2D(7, 1)])
    mObs1 = util.Polygon([util.Point2D(4, 5.5), util.Point2D(6, 4.8), util.Point2D(5, 6.5)])
    
    m = Map(bound, statObs=[sObs1, sObs2, sObs3], mvObs=[mObs1])
    robot = Robot(util.Point2D(1, 4), util.Point2D(9, 4))
    robot.initialize()
    # Animate
    anim.animate(m, robot)
    