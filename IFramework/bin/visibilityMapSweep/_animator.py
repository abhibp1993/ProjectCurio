# -*- coding: utf-8 -*-
"""
Created on Sat Oct 17 22:18:43 2015

@author: AbhishekKulkarni
"""

import sys
import os
dirIFramework = os.path.dirname(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))
sys.path.append(dirIFramework)

import pygame



def _polyDraw(self, canvas, color=(255, 255, 0), scale=1.0):
    points = []
    for p in self.pSet:
        temp = p.toList()
        temp[0] *= scale
        temp[1] = canvas.get_height() - scale * temp[1]
        points.append(temp)
    
    pygame.draw.polygon(canvas, color, points)

def _nodeDraw(self, canvas, color=(255,255,255), scale=1.0):
    x = int(self.point.x * scale)
    y = int(canvas.get_height() - self.point.y * scale)
    pygame.draw.circle(canvas, color, (x, y), 3)

def _edgeDraw(self, canvas, color=(255,255,255), scale=1.0):
    p1 = self.edge.p1.toList()
    p2 = self.edge.p2.toList()
    
    p1[0] = int(p1[0] * scale)
    p2[0] = int(p2[0] * scale)
    p1[1] = int(canvas.get_height() - p1[1] * scale)
    p2[1] = int(canvas.get_height() - p2[1] * scale)
    
    pygame.draw.line(canvas, (255,255,255), p1, p2, 1)
    
def _edgeDraw2(self, canvas, color=(255,255,255), scale=1.0):
    for n in self.E.keys():
        p1 = n.point.toList()
        p1[0] = int(p1[0] * scale)
        p1[1] = int(canvas.get_height() - scale * p1[1])
        
        for m in self.E[n]:
            p2 = m.point.toList()
            p2[0] = int(p2[0] * scale)
            p2[1] = int(canvas.get_height() - scale * p2[1])
            
            pygame.draw.line(canvas, color, p1, p2, 1)

def _graphDraw(self, canvas, color=(255,255,255), scale=1.0):
    scale = 40
    #print 'drawing %d points'%len(self.V)
    for n in self.V:
        n.draw(canvas, scale=40)
    
    for n in self.E.keys():
        p1 = n.point.toList()
        p1[0] = int(p1[0] * scale)
        p1[1] = int(canvas.get_height() - scale * p1[1])
        
        for m in self.E[n]:
            p2 = m.point.toList()
            p2[0] = int(p2[0] * scale)
            p2[1] = int(canvas.get_height() - scale * p2[1])
            
            pygame.draw.line(canvas, color, p1, p2, 1)

def _pathDraw(self, canvas, color=(0, 255, 255), scale=40.0):
    n = len(self.points)
    for i in range(n-1):
        p1 = self.points[i].toList()
        p2 = self.points[i+1].toList()
        
        p1[0] = int(scale * p1[0])
        p2[0] = int(scale * p2[0])
        p1[1] = int(canvas.get_height() - scale * p1[1])
        p2[1] = int(canvas.get_height() - scale * p2[1])
        
        pygame.draw.line(canvas, color, p1, p2, 2)

def animate(myMap, robot):
    pygame.init()
    screen = pygame.display.set_mode((400, 400))
    clock = pygame.time.Clock()
    done = False
    
    while not done: 
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                done = True
                
        try: 
            myMap.transduce([None])
            visGraph = robot.step(myMap)
            #print 'visGraph has %d nodes'%len(visGraph.V)
            #print type(path)
            
            screen.fill((0, 0, 0))
            
            try:
                myMap.draw(screen)
                visGraph.draw(screen)
                #path.draw(screen)
                #nextGoal.draw(screen, color=(0,255,0))
            except Exception, ex:
                print ex.message
            
            if robot.done(): done = True
            
            pygame.display.flip()
            clock.tick(10)
        except:
            pygame.display.quit()
        
    print 'goal reached'
    pygame.display.quit()
