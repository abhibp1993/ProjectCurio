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

__version__ = '2.0'

FLOAT_PRECISION = 4     # No. of Decimal Places
_CCW = 'ccw'
_CW = 'cw'
_COLLINEAR = 'collinear'

print 'importing util ' + __version__ + '...'



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
                
            if homoP[2][0] != 1:
                self.npPoint = np.array([[float(homoP[0][0])/homoP[2][0]], [float(homoP[1][0])/homoP[2][0]], [1]])
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
            self.mag = r
            self.arg = theta
            self.p2 = Point2D(p1.x + r*math.cos(theta), p1.y + r*math.sin(theta))
        elif isinstance(r, (float, int)) and isinstance(theta, (float, int)) and p2 == None and p1 == None:
            self.p1 = ORIGIN
            self.mag = r
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
        if (o1 == Vector2D._COLLINEAR and (vec.p1 in self)): return True
        if (o2 == Vector2D._COLLINEAR and (vec.p2 in self)): return True
        if (o3 == Vector2D._COLLINEAR and (self.p1 in vec)): return True
        if (o4 == Vector2D._COLLINEAR and (self.p2 in vec)): return True
        
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
        
        if ((lhs > rhs) and (not (abs(lhs - rhs) < eps))):   return Vector2D._CCW
        elif ((lhs < rhs) and (not (abs(lhs - rhs) < eps))): return Vector2D._CW
        elif abs(lhs - rhs) < eps:                           return Vector2D._COLLINEAR
            
    
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
        return self.turn(point) == Vector2D._COLLINEAR
    
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

    def __contains__(self, point):
        """ Checks if point lies on the vector. (Not just collinearity) """
        assert isinstance(point, Point2D), 'point must be instance of util.Point2D'
        return (self.collinear(point) and \
                point.x <= max(self.p1.x + Vector2D.EPS, self.p2.x + Vector2D.EPS, self.p1.x - Vector2D.EPS, self.p2.x - Vector2D.EPS) and \
                point.x >= min(self.p1.x + Vector2D.EPS, self.p2.x + Vector2D.EPS, self.p1.x - Vector2D.EPS, self.p2.x - Vector2D.EPS) and \
                point.y <= max(self.p1.y + Vector2D.EPS, self.p2.y + Vector2D.EPS, self.p1.y - Vector2D.EPS, self.p2.y - Vector2D.EPS) and \
                point.y >= min(self.p1.y + Vector2D.EPS, self.p2.y + Vector2D.EPS, self.p1.y - Vector2D.EPS, self.p2.y - Vector2D.EPS))
                
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
    
    def intersect(self, poly):
        """
        Computes if the polygon, poly, intersects self. 
        
        Running Time: O(mn), m, n are number of vertices in 2 polygons respectively.
        """
        assert isinstance(poly, Polygon), 'poly must be a util.Polygon instance'
        
        for e in self.edges:
           for d in poly.edges: 
               if d.intersect(e): return True
        
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
        
        
        
class Transform(object):
    """
    Represents a 2D transformation matrix. 
    Note: Composition is done in Rotation --> Translation order.
    """
    def __init__(self, translate=zeroVector, rotate=0.0):
        self.translate = translate
        self.rotate = rotate   

        
    @property    
    def transformMatrixRT(self):
        """
        Returns 3x3 Transformation Matrix
        RT: rotation followed by translation
        
        Note: while composing matrices, (A.(Bx)) = (B.A)x
        """
        return self.translationMatrix.dot(self.rotationMatrix)
                         
    @property    
    def transformMatrixTR(self):
        """
        Returns 3x3 Transformation Matrix
        TR: translation followed by rotation
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
        if      isinstance(obj, Point2D):   return self._applyToPoint(obj)
        elif    isinstance(obj, Vector2D):  return self._applyToVector(obj)
        elif    isinstance(obj, Pose):      return self._applyToPose(obj)
        elif    isinstance(obj, Transform): return self._applyToTransform(obj)
        elif    isinstance(obj, list):      return [self*o for o in obj]
        elif    isinstance(obj, tuple):     return (self*o for o in obj)
        else: raise AttributeError('Transform cannot be applied on ' + str(type(obj)))
            
    def __str__(self):
        return 'translate: ' + str(self.translate) + ' rotate: ' + str(self.rotate)
        
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


