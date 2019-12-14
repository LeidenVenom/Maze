# -*- coding: utf-8 -*-
"""
Created on Thu Dec 12 15:31:48 2019

@author: LeidenVenom
"""

import numpy as np
import random

R = np.matrix([[0, -1],
              [1, 0]])

r = np.array([[0], [1]])
u = R * r
l = R * u
d = R * l

class Node():
    
    def __init__(self, row, col, N):
        self.posi = np.array([[row],
                             [col]])
        self.near = [None, None, None, None]
        
        self.judged = False
        
        self.ID = N * row + col
        
def I2S(D, d):
    for i in range(D - len(d)):
        d = '0' + d
    return d


def P2S(D, p):
    for i in range(D - len(p)):
        p = ' ' + p
    return p


class Maze():
    def __init__(self, N):
        self.N = N
        m = []
        for i in range(N):
            m.append([])
            for j in range(N):
                m[i].append(Node(i, j, N))
        M = []
        for nodes in m:
            M += nodes
        for i in range(0, len(M)):
            for j in range(i + 1, len(M)):
                if np.linalg.norm(M[i].posi - M[j].posi) == 1 and random.choice([True, False]) == True:
                    p = M[j].posi - M[i].posi
                    if VecEqu(p, r):
                        M[i].near[0] = M[j]
                        M[j].near[2] = M[i]
                    if VecEqu(p, u):
                        M[i].near[1] = M[j]
                        M[j].near[3] = M[i]
                    if VecEqu(p, l):
                        M[i].near[2] = M[j]
                        M[j].near[0] = M[i]
                    if VecEqu(p, d):
                        M[i].near[3] = M[j]
                        M[j].near[1] = M[i]
            self.m = m
            self.M = M
            
    def show(self):
          N = self.N
          m = self.m
          D = len(str(N**2 - 1))
          for i in range(N):
              s = ''
              for j in range(N):
                  s += I2S(D, str(N * i + j))
                  if j == N - 1:
                      continue
                  if m[i][j].near[0] != None:
                      s += '-'
                  else:
                      s += ' '
              print(s)
              s = ''
              if i == N - 1:
                  continue
              for j in range(N):
                  if m[i][j].near[3] != None:
                      s += P2S(D, '|')
                  else:
                      s += P2S(D, '')
                  if j == N - 1:
                      continue
                  s += ' '
              print(s)
              
    def connect(self):
        N = self.N
        m = self.m
        M = self.M
        coll = []
        for i in range(N):
            for j in range(N):
                if m[i][j].judged == True:
                    continue
                coll.append(FindEqu(m[i][j]))
        u0 = []
        s = random.choice(M)
        for c in coll:
            if s in c:
                u0 = u0 + c
                break
        for k in range(len(coll) - 1):
            near = []
            for c in coll:
                if c[0] not in u0 and isNextTo(u0, c):
                    near.append(c)
            u1 = random.choice(near)
            near = []
            for k0 in u0:
                for k1 in u1:
                    if np.linalg.norm(k1.posi - k0.posi) == 1:
                        near.append(([k0, k1]))
            node1, node2 = random.choice(near)
            p = node2.posi - node1.posi
            if VecEqu(p, r):
                node1.near[0] = node2
                node2.near[2] = node1
            if VecEqu(p, u):
                node1.near[1] = node2
                node2.near[3] = node1
            if VecEqu(p, l):
                node1.near[2] = node2
                node2.near[0] = node1
            if VecEqu(p, d):
                node1.near[3] = node2
                node2.near[1] = node1
            u0 += u1
        return self
    
    def FindRoute(self, ID1, ID2):
        D = len(str((self.N)**2 - 1))
        start = self.m[int(ID1 / self.N)][ID1 % self.N]
        end = self.m[int(ID2 / self.N)][ID2 % self.N]
        K = self.DijkstraM(start)
        distance = K[0][end.ID]
        routes = self.FindNode(K, end, D)
        return [distance, routes]  
           
    def FindNode(self, K, node, D):
        if len(K[1][node.ID]) == 0:
            return [I2S(D, str(node.ID))]
        coll = []
        for i in range(len(K[1][node.ID])):
            ss = self.FindNode(K, K[1][node.ID][i], D)
            for j in range(len(ss)):
                ss[j] = ss[j] + '->' + I2S(D, str(node.ID))
            coll += ss
        return coll
    
    def Dijkstra(self, start):
        M = self.M
        N = self.N
        inf = N**2 + 1
        distance = []
        parent = []
        included = []
        for i in M:
            if VecEqu(i.posi, start.posi):
                included.append(True)
                distance.append(0)
                parent.append(None)
            elif i in start.near:
                included.append(False)
                distance.append(1)
                parent.append(start)
            else:
                included.append(False)
                distance.append(inf)
                parent.append(None)
        while(True):
            if False not in included:
                break
            for i in range(len(M)):
                if distance[i] < inf and included[i] == False:
                    included[i] = True
                    for node in M[i].near:
                        if node == None:
                            continue
                        k = node.ID
                        if included[k] == False:
                            newdis = distance[i] + 1
                            if newdis < distance[k]:
                                distance[k] = newdis
                                parent[k] = M[i]
        return [distance, parent]       
    
    def DijkstraM(self, start):
        M = self.M
        N = self.N
        inf = N**2 + 1
        distance = []
        parent = []
        included = []
        for i in M:
            if VecEqu(i.posi, start.posi):
                included.append(True)
                distance.append(0)
                parent.append([])
            elif i in start.near:
                included.append(False)
                distance.append(1)
                parent.append([start])
            else:
                included.append(False)
                distance.append(inf)
                parent.append([])
        while(True):
            if False not in included:
                break
            pre = []
            for i in range(len(M)):
                if distance[i] < inf and included[i] == False:
                    pre.append(M[i])
            for p in pre:
                for node in p.near:
                    if node == None:
                        continue
                    k = node.ID
                    if included[k] == False:
                        newdis = distance[p.ID] + 1
                        if newdis <= distance[k]:
                            distance[k] = newdis
                            parent[k].append(p)
            for p in pre:
                included[p.ID] = True
        return [distance, parent]     
                        
def VecEqu(p1, p2):
    for i in range(len(p1)):
        if p1[i][0] != p2[i][0]:
            return False
    return True
    

def FindEqu(node):
    equ = [node]
    node.judged = True
    for p in node.near:
        if p != None and p.judged == False:
            equ = equ + FindEqu(p)
    return equ
              
def isNextTo(equ1, equ2):
    for i in equ1:
        for j in equ2:
            if np.linalg.norm(i.posi - j.posi) == 1:
                return True
    return False

def divide():
    s = ''
    for i in range(40):
        s += '_'
    print(s)
    
divide()
n = int(input('Please enter the order of the maze: '))
divide()
maze = Maze(n)
print('\nRandom Maze:\n')
maze.show()
divide()
maze.connect()
print('\nRandom Maze Connected:\n')
maze.show()
divide()
while(True):
    start = int(input('Please enter the ID of the start node: '))
    end = int(input('Please enter the ID of the end node: '))
    divide()
    if start >= n**2 or end >= n**2:
        print('\nInput ID out of range.')
        divide()
        continue
    route = maze.FindRoute(start, end)
    print('\nThe ' + str(len(route[1])) + ' optimal path(s) which take(s) ' + str(route[0]) + ' step(s), are(is) printed below:\n')
    for s in route[1]:
        print(''.join(s))
    divide()
    if input('Do you want to quit? [y]/n: ') == 'y':
        divide()
        break
    divide()