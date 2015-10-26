# -*- coding: utf-8 -*-
"""
Created on Mon Oct 05 00:24:21 2015

Classes:
    - p 
    - Vector2D
    - Pose
    - Polygon
    - Transform

@author: AbhishekKulkarni (abhibp1993)
"""

import math
import numpy as np
from scipy.spatial import ConvexHull
import copy

__version__ = '2.0'

FLOAT_PRECISION = 4     # No. of Decimal Places
_CCW = 'ccw'
_CW = 'cw'
_COLLINEAR = 'collinear'

print 'importing util ' + __version__ + '...'


###########################################################################################
#Helper classes

class Node(object):
    def __init__(self, data, name):
        self.data = data
        self.name = name
        self.inNeighbours = list()
        self.outNeighbours = list()
        self.inDegree = 0
        self.outDegree = 0

        self.visited = False
        self.distance = float('inf')
                
    @property
    def nodeName(self):
        return self.name
        
    @nodeName.setter
    def nodeName(self,value):
        self.name  = value
        
    def Heuristic(self):
        return 0
        
    def __str__(self):
        return str(self.name)


class Edge(object):
    def __init__(self,source,destination,weight=1):
        self.source = source
        self.destination= destination
        self.weight = weight
        
    @property
    def edgeWeight(self):
        return self.weight
        
    @edgeWeight.setter
    def edgeWeight(self,value):
        self.weight = value


##########################################################################################
# User-Class Implementation

class Point2D(object):
    """
    Represents in Homogeneous Coordinates as column vector
    """
    _EPS = 1e-02
    
    def __init__(self, x=0.0, y=0.0, homoP = None, name='P'):
        """ 
        Constructor.
        
        @x: float or integer (Default: 0.0)
        @y: float or integer (Default: 0.0)
        @homoP: 1x3 or 3x1 numpy.array 
        """
        assert isinstance(x, (float, int)), 'x must be a float or int'
        assert isinstance(y, (float, int)), 'x must be a float or int'
        
        self._name = name
        if isinstance(homoP, type(None)):
            self.npPoint = np.array([[x], [y], [1]])
        else:
            assert homoP.shape in [(1, 3), (3, 1)], 'homogeneous coordinate must be np.array of size 3x1 or 1x3'
            if homoP.shape == (1, 3):
                homoP = homoP.transpose()
                
            if homoP[2][0] != 1 and homoP[2][0] != 0:
                self.npPoint = np.array([[float(homoP[0][0])/homoP[2][0]], [float(homoP[1][0])/homoP[2][0]], [1]])
            elif homoP[2][0] == 0:
                self.npPoint = np.array([[float('inf')], [float('inf')], [0]])
            else:
                self.npPoint = homoP
    
    @property 
    def name(self):
        return self._name
        
    @name.setter
    def name(self, string):
        self._name = string
    
    @property
    def x(self):
        return self.npPoint.tolist()[0][0]

    @property
    def y(self):
        return self.npPoint.tolist()[1][0]
    
    @property
    def positionVector(self):
        """ Returns a position vector to the self-point. """
        return Vector2D(ORIGIN, self)
    
    def toList(self):
        """ Return [x, y] list """
        return [self.x, self.y]
    
    def isNear(self, point, eps = _EPS):
        """ 
        Returns true if point is close this self-point.
        
        @point: Point2D instance
        @eps: float
        """        
        assert isinstance(point, Point2D), 'point must be instance of util.Point2D'
        assert isinstance(eps, float), 'eps must be a float'
        
        if self.distTo(point) < eps: return True
        return False
        
    def distTo(self, point):
        """ 
        Computes the distance of point from self-point
        
        @point: Point2D instance 
        """
        assert isinstance(point, Point2D), 'point must be instance of util.Point2D'
        return math.sqrt((self.x - point.x)**2 + (self.y - point.y)**2)
        
    def angleTo(self, point):
        """ 
        Computes the angle of point with respect to self-point
        @point: Point2D instance 
        """
        assert isinstance(point, Point2D), 'point must be instance of util.Point2D'
        return (math.atan2(point.y - self.y, point.x - self.x) % (2*math.pi))
    
    def __eq__(self, point):
        assert isinstance(point, Point2D), 'point must be instance of util.Point2D'
        return self.isNear(point)
    
    def __add__(self, point):
        assert isinstance(point, Point2D), 'point must be instance of util.Point2D'
        return Point2D(self.x + point.x, self.y + point.y)
        
    def __sub__(self, point):
        assert isinstance(point, Point2D), 'point must be instance of util.Point2D'
        return Point2D(self.x - point.x, self.y - point.y)
        
    def __transform__(self, T):
        assert isinstance(T, Transform), 'T must be instance of util.Transform'
        if T.isRT == True:
            p = T.transformMatrixRT.dot(self.npPoint)
        else:
            p = T.transformMatrixTR.dot(self.npPoint)
            
        return Point2D(homoP=p)
        
    def __str__(self):
        return self.name + '(' + str(round(self.x, FLOAT_PRECISION)) + ', ' + str(round(self.y, FLOAT_PRECISION)) + ')'
        
ORIGIN = Point2D()



class Vector2D(object):
    EPS = 1e-02
    
    def __init__(self, p1 = None, p2 = None, r=None, theta=None):
        """
        Constructor. 
        Vector2D can be defined in 2 ways.
            1. Using (p1, p2) as two points. p1: base, p2: tip
            2. Using (p1, r, theta). p1: base, r: length of vector, theta: angle with X+ axis of vector.
            
        @p1: Point2D instance 
        @p2: Point2D instance 
        @r: int or float. 
        @theta: int or float. Angle in radians
        
        """
        if isinstance(p1, Point2D) and isinstance(p2, Point2D) and r == None and theta == None:
            self.p1 = p1
            self.p2 = p2
            self.mag = p1.distTo(p2)
            self.arg = p1.angleTo(p2)
        elif isinstance(p1, Point2D) and isinstance(r, (float, int)) and isinstance(theta, (float, int)) and p2 == None:
            self.p1 = p1
            self.mag = abs(r)
            self.arg = theta
            self.p2 = Point2D(p1.x + r*math.cos(theta), p1.y + r*math.sin(theta))
        elif isinstance(r, (float, int)) and isinstance(theta, (float, int)) and p2 == None and p1 == None:
            self.p1 = ORIGIN
            self.mag = abs(r)
            self.arg = theta
            self.p2 = Point2D(r*math.cos(theta), r*math.sin(theta))
        else:
            self.p1 = ORIGIN
            self.p2 = ORIGIN
            self.mag = 0
            self.arg = 0
            raise AttributeError('Vector cannot be instantiated. See documentation for valid arguments.')
        
    @property
    def x(self):
        """ Returns length of x component of self-vector """
        return self.p2.x - self.p1.x

    @property
    def y(self):
        """ Returns length of x component of self-vector """
        return self.p2.y - self.p1.y
    
    @property
    def point(self):
        """ Returns length of x, y component of self-vector as Point2D instance"""
        return Point2D(self.x, self.y)
    
    @property
    def length(self):
        """ Returns length of vector """
        return self.mag
    
    @property
    def unitVector(self):
        """ Returns unit vector in direction of vector. """
        return Vector2D(p1=self.p1, r=1.0, theta=self.arg)
    
    @property
    def equation(self):
        """ Returns a numpy.array representing line in homogeneous coordinates """
        return np.cross(self.p1.npPoint.transpose(), self.p2.npPoint.transpose()).transpose()
        
    def normalize(self):
        """ Normalizes self-vector """
        self.mag = 1.0
        self.p2 = Point2D(self.p1.x + math.cos(self.arg), self.p1.y + math.sin(self.arg))
    
    def intersect(self, vec):
        """
        Returns True if self and vector intersect. 
        Caution: DOES NOT COMPUTE INTERSECTION POINT.
        
        @vec: Vector2D instance
        """
        assert isinstance(vec, Vector2D), 'vec must be instance of util.Vector2D'
        
        #orientations
        o1 = self.turn(vec.p1)
        o2 = self.turn(vec.p2)
        o3 = vec.turn(self.p1)
        o4 = vec.turn(self.p2)
        
        if (o1 != o2 and o3 != o4): return True
        if (o1 == _COLLINEAR and (vec.p1 in self)): return True
        if (o2 == _COLLINEAR and (vec.p2 in self)): return True
        if (o3 == _COLLINEAR and (self.p1 in vec)): return True
        if (o4 == _COLLINEAR and (self.p2 in vec)): return True
        
        return False
        
    def getIntersection(self, vec):
        """
        Returns the intersection point of self and vec.       
        @vec: Vector2D instance
        """
        assert isinstance(vec, Vector2D), 'vec must be instance of util.Vector2D'
        
        eqn1 = self.equation
        eqn2= vec.equation
        return Point2D(homoP=np.cross(eqn1.transpose(), eqn2.transpose()))
        
    def turn(self, point, eps = EPS):
        """
        Computes the orientation of point w.r.t. the vector.
        
        @point: Point2D instance
        @returns: {'ccw', 'cw', 'collinear'}
        """
        assert isinstance(point, Point2D), 'point must be instance of util.Point2D'
        
        lhs = (point.y - self.p1.y)*(self.p2.x - self.p1.x)
        rhs = (self.p2.y - self.p1.y)*(point.x - self.p1.x)
        
        if ((lhs > rhs) and (not (abs(lhs - rhs) < eps))):   return _CCW
        elif ((lhs < rhs) and (not (abs(lhs - rhs) < eps))): return _CW
        elif abs(lhs - rhs) < eps:                           return _COLLINEAR
            
    
    def isccw(self, point, eps = EPS):
        """
        Returns if the point is counter clockwise turn w.r.t. self.        
        
        @point: Point2D instance
        @eps: float 
        
        Remark: eps is used to ignore floating precision errors. If a point is epsilon
            close to vector, then it lies on the vector, hence ccw turn has no meaning.
        """
        assert isinstance(point, Point2D), 'point must be instance of util.Point2D'
        return self.turn(point) == Vector2D._CCW
        
    def iscw(self, point, eps = EPS):
        """
        Returns if the point is clockwise turn w.r.t. self.        
        
        @point: Point2D instance
        @eps: float        
        
        Remark: eps is used to ignore floating precision errors. If a point is epsilon
            close to vector, then it lies on the vector, hence cw turn has no meaning.
        """
        assert isinstance(point, Point2D), 'point must be instance of util.Point2D'
        return self.turn(point) == Vector2D._CW

    
    def collinear(self, point, eps = EPS):
        """
        Returns if the point is on the self-vector.
        
        @point: Point2D instance
        @eps: float 
        
        Remark: eps is used to ignore floating precision errors. If a point is epsilon
            close to vector, then it lies on the vector.
        """
        assert isinstance(point, Point2D), 'point must be instance of util.Point2D'
        return self.turn(point) == _COLLINEAR
    
    def dot(self, vec):
        """
        Computes the dot product of vectors
        @vec: Vector2D instance
        """
        assert isinstance(vec, Vector2D), 'vec must be instance of util.Vector2D'
        
        a = np.array([self.x, self.y])
        b = np.array([vec.x, vec.y])
        return np.dot(a, b)
        
    def cross(self, vec):
        """
        Computes the dot product of vectors
        @vec: Vector2D instance
        """
        assert isinstance(vec, Vector2D), 'vec must be instance of util.Vector2D'
        
        a = np.array([self.x, self.y])
        b = np.array([vec.x, vec.y])
        return np.cross(a, b)
        

    def __eq__(self, vec):
        assert isinstance(vec, Vector2D), 'vec must be instance of util.Vector2D'
        return (self.x == vec.x and self.y == vec.y)
    
    
    def __neg__(self, vec):
        assert isinstance(vec, Vector2D), 'vec must be instance of util.Vector2D'
        return Vector2D(p1=self.p2, p2=self.p1)
        
    def __add__(self, vec):
        assert isinstance(vec, Vector2D), 'vec must be instance of util.Vector2D'        
        return Vector2D(p1=self.p1, p2=Point2D(self.p2.x + vec.x, self.p2.y + vec.y))
    
    def __sub__(self, vec):
        assert isinstance(vec, Vector2D), 'vec must be instance of util.Vector2D'
        return Vector2D(p1=self.p1, p2=Point2D(self.p2.x - vec.x, self.p2.y - vec.y))
    
    def __mul__(self, obj):
        """
        Scaling or Dot product
        """
        print 'multiplying...'
        if isinstance(obj, Vector2D):
            return self.dot(obj)
        elif isinstance(obj, (float, int)):
            l = self.mag * obj
            return Vector2D(p1=self.p1, r=l, theta=self.arg)
        else:
            raise AssertionError('Multiplication with vector is defined with scalar or another vector only.')
    
    def __rmul__(self, obj):
        return self.__mul__(obj)
        
    def __pow__(self, vec):
        """
        Cross Product
        """
        assert isinstance(vec, Vector2D), 'vec must be instance of util.Vector2D'
        return self.cross(vec)
    
    def __lt__(self, point):
        """
        is turning ccw??
        """
        assert isinstance(point, Point2D), 'point must be instance of util.Point2D'
        return self.isccw(point)

    def __gt__(self, point):
        """
        is turning ccw??
        """
        assert isinstance(point, Point2D), 'point must be instance of util.Point2D'
        return self.iscw(point)

    def __contains__(self, point):
        """ Checks if point lies on the vector. (Not just collinearity) """
        assert isinstance(point, Point2D), 'point must be instance of util.Point2D'
        return (self.collinear(point) and \
                point.x <= max(self.p1.x + Vector2D.EPS, self.p2.x + Vector2D.EPS, self.p1.x - Vector2D.EPS, self.p2.x - Vector2D.EPS) and \
                point.x >= min(self.p1.x + Vector2D.EPS, self.p2.x + Vector2D.EPS, self.p1.x - Vector2D.EPS, self.p2.x - Vector2D.EPS) and \
                point.y <= max(self.p1.y + Vector2D.EPS, self.p2.y + Vector2D.EPS, self.p1.y - Vector2D.EPS, self.p2.y - Vector2D.EPS) and \
                point.y >= min(self.p1.y + Vector2D.EPS, self.p2.y + Vector2D.EPS, self.p1.y - Vector2D.EPS, self.p2.y - Vector2D.EPS))
                
    
    def __transform__(self, T):
        assert isinstance(T, Transform), 'T must be instance of util.Transform'
        tmpV = Vector2D(p1=ORIGIN, p2=self.p2-self.p1)  #translate vector to Origin
        nP1 = T * tmpV.p1
        nP2 = T * tmpV.p2
        return Vector2D(p1=(self.p1+nP1), p2=(self.p1+nP2))
        
    def __str__(self):
        return 'V[' + str(self.p1) + ', ' + str(self.p2) + ']'

zeroVector = Vector2D(Point2D(), Point2D())



class Pose(object):
    """
    Represents the Pose of the robot - (x, y, theta)
    """
    _EPS_DIST = 1e-02
    _EPS_ANGLE = 1e-02
    
    def __init__(self, x=None, y=None, theta=0.0, point=None):
        """
        Constructor.
        Instantiation can be done in two ways - 
            1. (x, y, theta)
            2. (point, theta)
        """        
        if isinstance(x, (float, int)) and isinstance(y, (float, int)) and point == None:
            self.npPose = np.array([[x], [y], [theta]])
        elif isinstance(point, Point2D) and x == None and y == None:
            self.npPose = np.array([[point.x], [point.y], [theta]])
        else:
            self.npPose = np.array([[0], [0], [0]])
            raise AttributeError('Instantiation of Pose failed. Check documentation for valid inputs')

    @property
    def x(self):
        """ returns x coordinate of pose """
        return self.npPose[0][0]

    @property
    def y(self):
        """ returns y coordinate of pose """
        return self.npPose[1][0]

    @property
    def theta(self):
        """ returns heading angle of pose """
        return self.npPose[2][0]
        
    @property
    def point(self):
        """ returns position represented by the Pose. """
        return Point2D(x=self.x, y=self.y)
        
    @property
    def homogeneousPoint(self):
        return np.array([[self.x], [self.y], [1]])
    
    @property
    def transform(self):
        """ Returns the transformation corresponding to Pose """
        T = Transform()
        T.translate = Vector2D(ORIGIN, Point2D(x=self.x, y=self.y))
        T.rotate = self.theta
        return T
        

    def applyTransform(self, T):
        """ 
        Applies transformation T to self-Pose 
        @T: util.Transform instance.
        """
        assert isinstance(T, Transform), 'T must be an instance of util.Transform'
        self.npPose = (T * self).npPose
        
    def distTo(self, pose):
        """ 
        Computes euclidean distance between self and pose 
        @pose: util.Pose instance
        """
        assert isinstance(pose, Pose), 'pose must be instance of Pose.'
        return np.linalg.norm(self.npPose[0:2] - pose.npPose[0:2])
    
    def isNear(self, pose, distEps = _EPS_DIST, angleEps = _EPS_ANGLE):
        """
        Computes if the pose is close to self.
        @pose: util.Pose instance
        """
        assert isinstance(pose, Pose), 'pose must be instance of Pose.'
        return ( self.distance(pose) < distEps) and abs(self.theta - pose.theta) < angleEps
        
    def toList(self):
        """ Returns Pose as a 3-list: [x, y, theta] """
        return self.npPose.transpose().tolist()[0]
        
    def toTuple(self):
        """ Returns Pose as a 3-tuple: (x, y, theta) """
        return tuple(self.npPose.transpose().tolist()[0])

    def __add__(self, pose):
        """ Adds self and pose """
        assert isinstance(pose, Pose), 'pose must be instance of Pose.'
        return Pose(x=self.x+pose.x, \
                    y=self.y+pose.y, \
                    theta=(self.theta+pose.theta)%(2*math.pi))
    
    def __sub__(self, pose):
        """ Subtracts self and pose """
        assert isinstance(pose, Pose), 'pose must be instance of Pose.'
        return Pose(x=self.x-pose.x, \
                    y=self.y-pose.y, \
                    theta=(self.theta-pose.theta)%(2*math.pi))

    def __str__(self):
        return 'Pose(' + str(round(self.x, FLOAT_PRECISION)) + ', ' + str(round(self.y, FLOAT_PRECISION)) + ', ' + str(round(math.degrees(self.theta), FLOAT_PRECISION)) + ')'
        
        
    
class Polygon(object):
    def __init__(self, pSet = None, poly = None):
        """
        Constructor.
        Instantiation may be done using two ways:
            1. pSet: list or tuple of Point2D instances.
            2. poly: another instance of Polygon.
        """
        if poly == None:
            assert isinstance(pSet, (list, tuple)), 'pSet must be a list or tuple'
            assert len(pSet) >= 3, 'At least 3 points required to define Polygon'
            assert not(False in [isinstance(p, Point2D) for p in pSet]), 'pSet must be set of util.Point2D instances'
            self.pSet = pSet
        elif pSet == None:
            assert isinstance(poly, Polygon), 'poly must be a util.Polygon instance'
            self.pSet = [p for p in poly.pSet]
        else:
            raise AttributeError('Instantiation Failed. Unacceptable set of arguments received.')
        
        #self._removeRedundantPoints()   #remove redundant points, if any.
        self._updateEdges()             # Generate edges variable, and fill it up.

    @property
    def isConvex(self):
        """
        Determines if the polygon is convex or not.
        Algorithm checks if all points in polygon lie on convex hull formed using these points.
        If yes, it's convex polygon, else - it's not!!
        
        Running Time: O(nlogn), using scipy's implementation of convex hull computation.
        [Note: Assumes no redundant points exist, i.e. no three consecutive vertices are collinear.]
        """
        points = [p.npPoint for p in self.pSet]
        hull = ConvexHull(points)
        if len(self.pSet) == len(hull.vertices):
            return True
        
        return False
        
    @property
    def area(self):
        raise NotImplementedError
    
    @property
    def vertices(self):
        return self.pSet

    @property
    def centroid(self):
        x = sum([p.x for p in self.pSet])/len(self.pSet)
        y = sum([p.y for p in self.pSet])/len(self.pSet)
        return Point2D(x, y)
        
        
    def _updateEdges(self):
        self.edges = []
        for i in range(-1, len(self.pSet)-1):
            self.edges.append(Vector2D(self.pSet[i], self.pSet[i+1]))
        
    def _removeRedundantPoints(self):
        """ 
        Removes any redundant point from vertex set. A redundant point is that, which 
        is collinear with other 2 vertices.  
        
        Running Time: O(n), n: number of vertices of polygon
        """
        redundant = []
        for i in range(-2, len(self.pSet)-3):
            p1, p2, p3 = self.pSet[i:i+3]
            if turn(p1, p2, p3) == _COLLINEAR: redundant.append(p2)
        
        for p in redundant:
            self.pSet.remove(p)
                    
    def _isInteriorPoint(self, point):
        """ 
        Checks if point lies inside the polygon. Works for convex, and non-convex polygons.
        Implements Ray-Casting Algorithm 
        
        Running Time: O(n), n is number of vertices of polygon.
        """
        assert isinstance(point, Point2D), 'point must be instance of util.Point2D'
        
        #select a point @ x= 2*max(all points.x)
        rayTip = Point2D(y=point.y)
        rayTip.x = 2 * max([p.x for p in self.pSet])
        ray = Vector2D(point, rayTip)
        
        # count number of intersections
        intersectionCount = 0
        for e in self.edges:
            if ray.intersect(e): intersectionCount += 1 
            
        if intersectionCount % 2 == 1: return True
        return False
    
    def intersect(self, obj):
        """
        Computes if the polygon, poly, intersects self. 
        
        Running Time: O(mn), m, n are number of vertices in 2 polygons respectively.
        """
        assert isinstance(obj, (Vector2D, Polygon)), 'poly must be a util.Vector2D or util.Polygon instance'
        
        if isinstance(obj, Polygon):
            for e in self.edges:
               for d in obj.edges: 
                   if d.intersect(e): return True
            
            return False
        else:
            for d in self.edges: 
                   if d.intersect(obj): return True
                  
            return False                  
            
        
    def getIntersection(self, poly):
        assert isinstance(poly, Polygon), 'poly must be a util.Polygon instance'
        raise NotImplementedError
    
    def union(self, poly):
        assert isinstance(poly, Polygon), 'poly must be a util.Polygon instance'
        raise NotImplementedError

    def applyTransform(self, T):
        assert isinstance(T, Transform), 'T must be an instance of util.Transform'
        self.pSet = T * self.pSet
    
    def __contains__(self, obj):
        if    isinstance(obj, Point2D):  return self._isInteriorPoint(obj)
        elif  isinstance(obj, Vector2D): return self._isInteriorPoint(obj.p1) and self._isInteriorPoint(obj.p2)
        elif  isinstance(obj, Polygon):  return self._isInteriorPoint(obj.pSet)
        elif  isinstance(obj, list):     return not (False in [p in self for p in obj])
        elif  isinstance(obj, tuple):    return not (False in [p in self for p in obj])
        else: TypeError('Invalid input for "in" function')
            
    def __add__(self, poly):
        """ Union of two polygons """
        assert isinstance(poly, Polygon), 'poly must be a util.Polygon instance'
        return self.union(poly)
    
    def __mul__(self, poly):
        """ Intersection of two polygons """
        assert isinstance(poly, Polygon), 'poly must be a util.Polygon instance'
        return self.getIntersection(poly)
        
    def __transform__(self, T):
        G = self.centroid
        newPoints = [P-G for P in self.pSet]
        vectors = [Vector2D(ORIGIN, P) for P in newPoints]
        
        newpSet = []
        for v in vectors:
            newV = T * v
            newpSet.append(newV.p2)
        
        points = [p+G for p in newpSet]
        return Polygon(points)
        

class Circle(object):
    _EPS = 1e-02
    
    def __init__(self, center=Point2D(), rad=1.0):
        self.C = center
        self.r = rad
    
    def on(self, point, eps=_EPS):
        if abs(self.C.distTo(point) - self.r) < eps:
            return True
        
        return False
    
    def isInterior(self, point, eps=_EPS):
        if (self.r - self.C.distTo(point)) > eps:
            return True
        
        return False
    
    def isExterior(self, point, eps=_EPS):
        return (not self.isInterior(point)) and (not self.on(point))
    
    def tangent(self, point):
        """
        Returns the tangent vectors from external point to self-circle.
        
        @point: util.Point2D instance
        @returns: 2-list of util.Vector2D instances 
        
        Remark: 
        - First vector represents the tangent oriented ccw w.r.t. PC vector,
          where P is external point, C is center of circle.
        - The point of contact can be extracted by accessing "p2" 
          property of returned vectors.
        """
        assert isinstance(point, Point2D), 'point must be instance of util.Point2D class'
        
        vecPC = Vector2D(p1=point, p2=self.C)
        angle = math.asin(self.r/point.distTo(self.C))
        T = Transform()
        
        T.rotate = angle
        vecPT1 = T * vecPC
        
        T.rotate = -angle
        vecPT2 = T * vecPC
        
        vecPT1.normalize()
        vecPT2.normalize()
        vecPT1 = vecPT1 * (point.distTo(self.C) * math.cos(angle))
        vecPT2 = vecPT2 * (point.distTo(self.C) * math.cos(angle))

        return [vecPT1, vecPT2]
        
    def supportingTangent(self, circle, eps=_EPS):
        assert isinstance(circle, Circle), 'circle must be instance of util.Circle class'
        
#        if abs(circle.r - self.r) > eps:
#            C1C2 = Vector2D(p1=circle.r, p2=self.r)
#            P1 = Vector2D(p1=circle.C, r=circle.r, theta=(C1C2.arg+math.pi/2)).p2
#            P2 = Vector2D(p1=self.C,   r=self.r,   theta=(C1C2.arg+math.pi/2)).p2
#            return Vector2D(p1=P1, p2=P2)
#        
#        elif circle.r < self.r:
#            smallCircle = circle
#            bigCircle   = self
#        
#        else: #circle.r > self.r
#            smallCircle = self
#            bigCircle   = circle
#            
#        P = 
                    
        
class Transform(object):
    """
    Represents a 2D transformation matrix. 
    Note: Composition is done in Rotation --> Translation order.
    """
    def __init__(self, translate=zeroVector, rotate=0.0, applyRotThenTrans=True):
        self.translate = translate
        self.rotate = rotate   
        self.isRT = applyRotThenTrans

        
    @property    
    def transformMatrixRT(self):
        """
        Returns 3x3 Transformation Matrix
        RT: rotation followed by translation, i.e. object is rotated first then translated
        
        Note: while composing matrices, (A.(Bx)) = (B.A)x
        """
        return self.translationMatrix.dot(self.rotationMatrix)
                         
    @property    
    def transformMatrixTR(self):
        """
        Returns 3x3 Transformation Matrix
        TR:  translation followed by rotation, i.e. object is translated first then rotated
        """
        return self.rotationMatrix.dot(self.translationMatrix)
                         
    @property
    def translationMatrix(self):
        """ Returns 2D translation matrix """
        return np.array([[1, 0, self.translate.x], \
                         [0, 1, self.translate.y], \
                         [0, 0,                1]])
    
    @property    
    def rotationMatrix(self):
        """
        Returns 3x3 Transformation Matrix
        """
        return np.array([[math.cos(self.rotate), -math.sin(self.rotate), 0], \
                         [math.sin(self.rotate),  math.cos(self.rotate), 0], \
                         [                    0,                     0,  1]])
                         
    def _applyToPoint(self, point):
        """ Applies transform to point (First Rotate, then Translate) """
        assert isinstance(point, Point2D), 'point must be instance of util.Point2D'
        p = self.transformMatrixRT.dot(point.npPoint)        
        return Point2D(homoP=p)
    
    def _applyToPose(self, pose):
        """ Applies transform to pose (First Rotate, then Translate) """
        assert isinstance(pose, Pose), 'pose must be instance of util.Pose'
        
        tmp = pose.transform.transformMatrixRT.dot(self.transformMatrixRT)
        nx = tmp[0][2]
        ny = tmp[1][2]
        ntheta = math.atan2(tmp[1][0], tmp[0][0]) % (2 * math.pi)
        
        return Pose(x=nx, y=ny, theta=ntheta)
        
#        t = (pose.theta + self.rotate) % (2 * math.pi)
#        self.rotate = t
#        tmp = self.transformMatrixTR.dot(pose.homogeneousPoint)
#        print self.transformMatrixTR
#        return Pose(x=tmp[0][0], y=tmp[1][0], theta=t)
        
    def _applyToVector(self, vec):
        assert isinstance(vec, Vector2D), 'vec must be instance of util.Vector2D'
        
        p1 = self._applyToPoint(vec.p1)
        p2 = self._applyToPoint(vec.p2)
        return Vector2D(p1, p2)
        
    def _applyToTransform(self, T):
        """ Intricate details of sequencing """
        assert isinstance(T, Transform), 'T must be an instance of util.Transform'
        
        newT = T.transformMatrixRT.dot(self.transformMatrixRT)
        trans = Vector2D(ORIGIN, Point2D(x=newT[0][2], y=newT[1][2]))
        rot = self.rotate + T.rotate
        return Transform(translate=trans, rotate=rot)
    
    def __mul__(self, obj):
        if      isinstance(obj, Point2D):   return obj.__transform__(self) #self._applyToPoint(obj)
        elif    isinstance(obj, Vector2D):  return obj.__transform__(self) #self._applyToVector(obj)
        elif    isinstance(obj, Pose):      return self._applyToPose(obj)
        elif    isinstance(obj, Transform): return self._applyToTransform(obj)
        elif    isinstance(obj, Polygon):   return obj.__transform__(self)
        elif    isinstance(obj, list):      return [self*o for o in obj]
        elif    isinstance(obj, tuple):     return (self*o for o in obj)
        else: raise AttributeError('Transform cannot be applied on ' + str(type(obj)))
            
    def __str__(self):
        return 'translate: ' + str(self.translate) + ' rotate: ' + str(round(self.rotate, FLOAT_PRECISION))
        


class Graph(object):
    def __init__(self,allowDuplication=True):
        self.nodeList = list()
        self.edgeList = list()
        self.allowDuplication = allowDuplication
        
    def addNode(self,data,name):
        if self.allowDuplication == False:
            for node in self.nodeList:
                if node.data == data:
                    print "Duplication of data not allowed returning node with same data \n"
                    return node
        for node in self.nodeList:
            if node.nodeName == name:
                print "Duplicat names not allowed returning node with same name \n"
                return node
                
        newNode = Node(data,name)
        self.nodeList.append(newNode)
        return newNode
        
    def addEdge(self,source,destination,weight=1):
        newEdge = Edge(source,destination,weight)
        self.edgeList.append(newEdge)
        source.inNeighbours.append(destination)
        destination.inNeighbours.append(source)
        
        source.inDegree += 1
        destination.inDegree += 1
        
        #Indegree outdegree for non directional graph is same
        source.outDegree = source.inDegree
        destination.outDegree = destination.inDegree
        
        source.outNeighbours = source.inNeighbours
        destination.outNeighbours = destination.inNeighbours
        
    def getEdge(self,source,destination):
        for edge in self.edgeList:
            if((edge.source.nodeName == source.nodeName and edge.destination.nodeName == destination.nodeName) or (edge.source.nodeName == destination.nodeName and edge.destination.nodeName == source.nodeName)):
                return edge

    def reset(self):
        for node in self.nodeList:
            node.visited = False
            node.distance = float('inf')
                
        


class DiGraph(object):
    def __init__(self):
        self.nodeList = list()
        self.edgeList = list()
        
    def addNode(self,data,name):
        if self.allowDuplication == False:
            for node in self.nodeList:
                if node.data == data:
                    print "Duplication of data not allowed returning node with same data \n"
                    return node
                    
        for node in self.nodeList:
            if node.nodeName == name:
                print "Duplicat names not allowed returning node with same name \n"
                return node
                
        newNode = Node(data,name)
        self.nodeList.append(newNode)
        return newNode
        
    def addEdge(self,source,destination,weight):
        newEdge = Edge(source,destination,weight)
        self.edgeList.append(newEdge)
        
        source.outNeighbours.append(destination)
        destination.inNeighbours.append(source)        

        source.outDegree += 1
        destination.inDegree += 1
        
        
    def getEdge(self,source,destination):
        for edge in self.edgeList:
            if(edge.source.nodeName == source.nodeName and edge.destination.nodeName == destination.nodeName):
                return edge


###########################################################################################
#Helper Functions

def greedyPop(nodeList):
   
    minNode = nodeList[0]
    for node in nodeList:
        if minNode.distance > node.distance:
            minNode = node
            
    return minNode

###########################################################################


def turn(p1, p2, p3, eps = (10**FLOAT_PRECISION)):
    """ 
    Computes the turn from p1->p2 to p3 as clockwise, counter-clockwise or straight.
    
    @p1: Point2D instance
    @p2: Point2D instance
    @p3: Point2D instance
    """
    lhs = (p3.y - p1.y)*(p2.x - p1.x)
    rhs = (p2.y - p1.y)*(p3.x - p1.x)
    
    if ((lhs > rhs) and (not (abs(lhs - rhs) < eps))):   return _CCW
    elif ((lhs < rhs) and (not (abs(lhs - rhs) < eps))): return _CW
    elif abs(lhs - rhs) < eps:                           return _COLLINEAR
      
def Rot(theta):
    return Transform(rotate=theta)


def aStar(Graph,start,goal):
    path = list()
    totalCost = 0
    previousNode = dict()
    
    start.distance = 0
    nodeQueue = copy.copy(Graph.nodeList)

    while(len(nodeQueue) != 0):
        minNode = greedyPop(nodeQueue)
        nodeQueue.remove(minNode)
        
        if minNode.nodeName == goal.nodeName:
            break
        
        for neighbour in minNode.outNeighbours:
            newEdge = Graph.getEdge(minNode,neighbour)
            newDist = minNode.distance + newEdge.weight + neighbour.Heuristic()
            if newDist < neighbour.distance:
                neighbour.distance = newDist
                previousNode[neighbour] = minNode
                

    tempNode = goal
    while(tempNode.nodeName != start.nodeName):
        path.append(tempNode)
        tempNode = previousNode[tempNode]
        
    path.append(start)
    path.reverse()
    totalCost = goal.distance
    Graph.reset()
    return (path,totalCost)


def dijkstra(Graph,start,goal):
    path = list()
    totalCost = 0
    previousNode = dict()
    
    start.distance = 0
    nodeQueue = copy.copy(Graph.nodeList)

    while(len(nodeQueue) != 0):
        minNode = greedyPop(nodeQueue)
        nodeQueue.remove(minNode)
        
        if minNode.nodeName == goal.nodeName:
            break
        
        for neighbour in minNode.outNeighbours:
            newEdge = Graph.getEdge(minNode,neighbour)
            newDist = minNode.distance + newEdge.weight
            if newDist < neighbour.distance:
                neighbour.distance = newDist
                previousNode[neighbour] = minNode
                

    tempNode = goal
    while(tempNode.nodeName != start.nodeName):
        path.append(tempNode)
        tempNode = previousNode[tempNode]
        
    path.append(start)
    path.reverse()
    totalCost = goal.distance
    Graph.reset()
    return (path,totalCost)

def DFS(start, path=[]):
  '''recursive depth first search from start'''
  path=path+[start]
  start.visited = True

  for node in start.outNeighbours:
    if not node.visited == True:
      path=DFS(node, path)
  return path


def checkConnectivity(Graph):
    travelList = DFS(Graph.nodeList[0])

    Graph.reset()
    if(len(Graph.nodeList) != len(travelList)):
        return False
    elif (len(Graph.nodeList) == len(travelList)):
        return True
    else:
        return None

def checkRechability(start, end, path = []):
    path = path + [start]
    start.visited = True

    if start == end:
        return (True,path)

    for node in start.outNeighbours:
        if node.visited != True: #avoid cycles
            newPath = checkRechability(node,end,path)
            if newPath != None:
                return (True,newPath)

    return (False,None)




