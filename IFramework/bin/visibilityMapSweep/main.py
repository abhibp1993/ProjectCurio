# -*- coding: utf-8 -*-
"""
Created on Mon Oct 19 22:19:01 2015

@author: abhibp1993
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
## GRAPH GENERATION

class Node(object):
    def __init__(self, point):
        self.point = point
        self.pathLength = float('inf')      #see if there's some way to make this infinity
        self.parent = None
        self.explored = -1
        self.heuristicVal = 0
        
    def getPathLength(self):
        return self.pathLength

    def getParent(self):
        return self.parent

    def getExplored(self):
        """
        -1: Unexplored, 0: Frontier, 1: Explored
        """
        return self.explored
    
    def setPathLength(self, length):
        self.pathLength = length

    def setParent(self, parentNode):
        self.parent = parentNode

    def setExplored(self, explored):
        """
        -1: Unexplored, 0: Frontier, 1: Explored
        """
        self.explored = explored
    
    def setHeuristic(self, value):
        self.heuristicVal = value

    def getHeuristic(self):
        return self.heuristicVal    
    
    def __eq__(self, obj):
        if obj.point == self.point: return True
        return False
        
    def __str__(self):
        return 'Node: ' + str(self.point)
        
Node.draw = anim._nodeDraw

class Edge(object):
    def __init__(self, srcNode, dstNode):
        self.srcNode = srcNode
        self.dstNode = dstNode
        self.edge = util.Vector2D(srcNode.point, dstNode.point)
        self.weight = srcNode.point.distTo(dstNode.point)
        
    def getWeight(self):
        return self.weight
    
    def __eq__(self, e):
        if self.srcNode == e.srcNode and self.dstNode == e.dstNode: return True
        return False

    def __str__(self):
        return "(" + str(self.srcNode) + ", " + str(self.dstNode) + ")"

Edge.draw = anim._edgeDraw

class Graph(object):
    def __init__(self, V = [], E = {}):
        self.V = V
        self.E = E

    def getNode(self, point):
        for v in self.V:
            if v.point == point:
                return v
        
        return None
                
    def addEdge(self, e):
        n1 = self.getNode(e.p1)
        n2 = self.getNode(e.p2)
        
        if not n2 in self.E[n1]:
            self.E[n1].append(n2)
        if not n1 in self.E[n2]:
            self.E[n2].append(n1)
        

    def addNode(self, n):
        self.V.append(n)
        self.E[n] = []
            
    def childrenOf(self, node):
        node = self.getNode(node.point)
        return self.E[node]
        
    def getWeight(self, src, dest):        
        return src.point.distTo(dest.point)
    
    def __str__(self):
        return "n(V) = %d, n(E) = %d"%(len(self.V), len(self.E))

Graph.draw = anim._graphDraw

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
    G = Graph()
    G.V = []
    G.E = {}

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
            print 'rotSweep started...'
            tmp = rotSweep(p, points[i+1:], polyEdges)
            print 'rotSweep ended...'
        except Exception, ex:
            print ex.message
            
        visEdges.extend(tmp)
        
    # All polygon edges are edges in visibility graph
    visEdges.extend(polyEdges)
    
    # Construct Graph
    #print 'points being added %d'%len(points)
    for p in points:
        G.addNode(Node(p)) 
        
    #G.E = [Edge(e) for e in visEdges]
    for e in visEdges:
        G.addEdge(e)  
        
    return G

#######################################################################
## A* Algorithm
 
def APop(nodeList):
    """
    returns the node with least pathLength
    """

    bestNode = nodeList[0]
    minlen = nodeList[0].getPathLength() + nodeList[0].getHeuristic()
    
    for n in nodeList:
        if n.getPathLength() + n.getHeuristic() < minlen:
            bestNode = n
            minlen = n.getPathLength() + n.getHeuristic()

    nodeList.remove(bestNode)
    return bestNode


def Heuristic(graph):
    """
    takes in a graph and applies the chosen Heuristic value for each Node.
    """
    pass    ##At present applies No Heuristic!

    
def AStar(graph, start, goal):
    assert isinstance(start, Node), 'start must be a Node'
    assert isinstance(goal, Node), 'goal must be a Node'
    assert isinstance(graph, Graph), 'graph must be a Graph'

    if len(graph.V) < 2:
        return []
    
    print '-------------'
    print 'Iteration of A* starting...'
    
    # Initialize search     
    start.parent = None
    start.pathLength = 0
    
    # Initialize frontier
    frontier = [start]
    
    loopCount = 0
    while(len(frontier) > 0 and loopCount < 100): 
        print '------'
        print loopCount
        
        print 'Frontier'
        for n in frontier:
            print '\t', n
            
        # Greedy Pop
        tmpNode = APop(frontier)
        print 'Poped', tmpNode
        
        
        # Check if goal node is explored and reached
        if tmpNode == goal:
            path = []
            n = tmpNode
            while (n != None):
                path.insert(0, n)
                n = n.parent
            print 'Path found: ', path
            return path
        
        # get all children of tmpNode
        children = graph.childrenOf(tmpNode)
        for c in children: print 'child of ', tmpNode, 'is', c
        tmpNode.explored = 1
        
        for child in children:
            newPathLen = tmpNode.pathLength + graph.getWeight(tmpNode, child)
            print 'Selected Child', child
            print '\t', 'new', newPathLen, 'old', child.pathLength
            if newPathLen <= child.pathLength and (child.explored in (0, -1)): 
                child.parent = tmpNode
                child.pathLength = newPathLen
                child.explored = 0
                if not child in frontier:
                    frontier.append(child)
                
                print '\tpushing ', child, 'on frontier'
                print '\tnew frontier length', len(frontier)
        
        
        loopCount += 1
    
    return []

    
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
        print 'Step 3: Computed Visibility Graph...'
    
#        print 'Step 4: Computing Optimum Path...'
#        path = AStar(g, Node(state), Node(self.goal))
#        print 'Step 5: Optimum Path Computed...'
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
    