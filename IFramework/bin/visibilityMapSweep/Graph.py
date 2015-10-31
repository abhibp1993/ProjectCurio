# -*- coding: utf-8 -*-
"""
Class Description:
    
"""
from Node import Node
from Edge import Edge

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
                    
        newNode = Node(data,name)
        self.nodeList.append(newNode)
        return newNode
        
    def addEdge(self,source,destination,weight):
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
