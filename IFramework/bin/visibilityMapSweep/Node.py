# -*- coding: utf-8 -*-

"""
Class description:

"""

class Node(object):
    def __init__(self, data, name):
        self.data = data
        self.name = name
        self.inNeighbours = list()
        self.outNeighbours = list()
        self.inDegree = 0
        self.outDegree = 0
        
    @property
    def nodeName(self):
        return self.name
        
    @nodeName.setter
    def nodeName(self,value):
        self.name  = value
        
#==============================================================================
#     @property
#     def uniqueData(self):
#         return self.allowDuplication
#         
#     @uniqueData.setter
#     def uniqueData(self,value):
#         if type(value) is bool:
#             self.allowDuplication = value
#         else:
#             raise TypeError
#==============================================================================
            
    def Heuristic(self):
        return 0
        
    def __str__(self):
        return str(self.name)
        