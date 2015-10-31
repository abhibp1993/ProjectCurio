# -*- coding: utf-8 -*-
"""
Created on Fri Oct 23 13:32:13 2015

@author: monica
"""

#================== Helper Functions ==================================================

def findMin(distance,nodeList):
    nodeList = nodeList
    
    minNode = nodeList[0]
    for node in nodeList:
        if distance[minNode] > distance[node]:
            minNode = node
            
    return minNode


#=========================== Graph Algorihtms =================================================

def Astar(Graph,start,goal):
    path = list()
    distance = dict()
    previousNode = dict()
    
    #Set all Node distance in Graph as infinity
    for node in Graph.nodeList:
        distance[node] = float('inf')
        
    #Distance of start Node = 0
    distance[start] = 0
    nodeQueue = Graph.nodeList
    
    while(len(nodeQueue) != 0):
        minNode = findMin(distance,nodeQueue)
        nodeQueue.remove(minNode)
        
        if minNode.nodeName == goal.nodeName:
            break
        
        for neighbour in minNode.outNeighbours:
            newEdge = Graph.getEdge(minNode,neighbour)
            newDist = distance[minNode] + newEdge.weight + neighbour.Heuristic()
            if newDist < distance[neighbour]:
                distance[neighbour] = newDist
                previousNode[neighbour] = minNode
                

    tempNode = goal
    while(tempNode.nodeName != start.nodeName):
        path.append(tempNode)
        tempNode = previousNode[tempNode]
        
    path.append(start)
    path.reverse()
    return path
            
        
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    