# -*- coding: utf-8 -*-
"""
Created on Mon Oct 05 00:24:21 2015

Classes:
    - Point2D
    - Vector2D
    - Pose
    - Transform

@author: AbhishekKulkarni
"""

import math
import numpy as np

FLOAT_PRECISION = 4     # No. of Decimal Places


print 'importing util modified...'



class Point2D(object):
    """
    Represents in Homogeneous Coordinates as column vector
    """
    
    def __init__(self, x=0.0, y=0.0):
        """ 
        Constructor.
        
        @x: float or integer (Default: 0.0)
        @y: float or integer (Default: 0.0)
        """
        assert isinstance(x, (float, int)), 'x must be a float or int'
        assert isinstance(y, (float, int)), 'x must be a float or int'
        self._point = np.array([[x], [y], [1]])
    
    @property
    def x(self):
        return self._point.tolist()[0][0]

    @property
    def y(self):
        return self._point.tolist()[1][0]
    
    @property
    def positionVector(self):
        """ Returns a position vector to the self-point. """
        return Vector2D(ORIGIN, self)
    
    def toList(self):
        """ Return [x, y] list """
        return [self.x, self.y]
    
    def isNear(self, point, eps = 1e-02):
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
    
    def __str__(self):
        return 'P(' + str(self.x) + ', ' + str(self.y) + ')'
        
ORIGIN = Point2D()



class Vector2D(object):
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
            self.mag = r
            self.arg = theta
            self.p2 = Point2D(p1.x + r*math.cos(theta), p1.y + r*math.sin(theta))
        else:
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
    def length(self):
        """ Returns length of vector """
        return self.mag
    
    @property
    def unitVector(self):
        """ Returns unit vector in direction of vector. """
        return Vector2D(p1=self.p1, r=1.0, theta=self.arg)
    
    def normalize(self):
        """ Normalizes self-vector """
        self.r = 1
        self.p2 = Point2D(self.p1.x + math.cos(self.arg), self.p1.y + math.sin(self.arg))
    
    def intersect(self, vec):
        """
        Returns True if self and vector intersect. 
        Caution: DOES NOT COMPUTE INTERSECTION POINT.
        
        @vec: Vector2D instance
        """
        assert isinstance(vec, Vector2D), 'vec must be instance of util.Vector2D'
        raise NotImplementedError
        
    def getIntersection(self, vec):
        """
        Returns the intersection point of self and vec.       
        @vec: Vector2D instance
        """
        assert isinstance(vec, Vector2D), 'vec must be instance of util.Vector2D'
        raise NotImplementedError
        
    def isccw(self, point, eps = 1e-02):
        """
        Returns if the point is counter clockwise turn w.r.t. self.        
        
        @point: Point2D instance
        @eps: float 
        
        Remark: eps is used to ignore floating precision errors. If a point is epsilon
            close to vector, then it lies on the vector, hence ccw turn has no meaning.
        """
        assert isinstance(point, Point2D), 'point must be instance of util.Point2D'
        
        lhs = (point.y - self.p1.y)*(self.p2.x - self.p2.x)
        rhs = (self.p2.y - self.p1.y)*(point.x - self.p2.x)
        return  ((lhs > rhs) and (not (lhs - rhs) < eps))
        
    def iscw(self, point, eps = 1e-02):
        """
        Returns if the point is clockwise turn w.r.t. self.        
        
        @point: Point2D instance
        @eps: float        
        
        Remark: eps is used to ignore floating precision errors. If a point is epsilon
            close to vector, then it lies on the vector, hence cw turn has no meaning.
        """
        assert isinstance(point, Point2D), 'point must be instance of util.Point2D'
        
        lhs = (point.y - self.p1.y)*(self.p2.x - self.p2.x)
        rhs = (self.p2.y - self.p1.y)*(point.x - self.p2.x)
        return  ((lhs < rhs) and (not (lhs - rhs) < eps))

    
    def on(self, point, eps = 1e-02):
        """
        Returns if the point is on the self-vector.
        
        @point: Point2D instance
        @eps: float 
        
        Remark: eps is used to ignore floating precision errors. If a point is epsilon
            close to vector, then it lies on the vector.
        """
        assert isinstance(point, Point2D), 'point must be instance of util.Point2D'
        
        lhs = (point.y - self.p1.y)*(self.p2.x - self.p2.x)
        rhs = (self.p2.y - self.p1.y)*(point.x - self.p2.x)
        return (lhs - rhs) < eps
    
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
        
        tempVec = vec
        if vec.p1 != self.p2:
            tempVec = vec - (vec.p2.positionVector - self.p2.positionVector)
        
        return Vector2D(p1=self.p1, p2=Point2D(self.p2.x + tempVec.x, self.p2.y + tempVec.y))
    
    def __sub__(self, vec):
        assert isinstance(vec, Vector2D), 'vec must be instance of util.Vector2D'
        
        tempVec = vec
        if vec.p1 != self.p2:
            tempVec = vec - (vec.p2.positionVector - self.p2.positionVector)
        
        return Vector2D(p1=self.p1, p2=Point2D(self.p2.x - tempVec.x, self.p2.y - tempVec.y))
    
    def __mul__(self, vec):
        """
        Dot product
        """
        assert isinstance(vec, Vector2D), 'vec must be instance of util.Vector2D'
        return self.dot(vec)
   
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

    def __contains__(self, vec):
        assert isinstance(vec, Vector2D), 'vec must be instance of util.Vector2D'
        raise NotImplementedError
        
    def __str__(self):
        return 'V[' + str(self.p1) + ', ' + str(self.p2) + ']'

zeroVector = Vector2D(Point2D(), Point2D())



class Pose(object):
    pass



class Transform(object):
    def __init__(self, translate=zeroVector, rotate=0.0, scale=1.0):
        self.translate = translate
        self.rotate = rotate  
        self.scale = scale

    @property    
    def transformMatrix(self):
        """
        Returns 3x3 Transformation Matrix
        """
        raise NotImplementedError
        
    def _applyToPoint(self, point):
        assert isinstance(point, Point2D), 'point must be instance of util.Point2D'
        raise NotImplementedError
    
    def _applyToPose(self, pose):
        assert isinstance(pose, Pose), 'pose must be instance of util.Pose'
        raise NotImplementedError
    
    def _applyToVector(self, vec):
        assert isinstance(vec, Vector2D), 'vec must be instance of util.Vector2D'
        raise NotImplementedError
    
    def __mul__(self, obj):
        if      isinstance(obj, Point2D):   return self._applyToPoint(obj)
        elif    isinstance(obj, Vector2D):  return self._applyToVector(obj)
        elif    isinstance(obj, Pose):      return self._applyToPose(obj)
        elif    isinstance(obj, Transform): raise NotImplementedError
        elif    isinstance(obj, list):      raise NotImplementedError
        elif    isinstance(obj, tuple):     raise NotImplementedError
        else: raise AttributeError('Transform cannot be applied on ' + str(type(obj)))
            
            