# -*- coding: utf-8 -*-
"""
Class Description:
    
"""

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
