import numpy as np
from random import random
import matplotlib.pyplot as plt
from matplotlib import collections  as mc
from collections import deque
from PIL import Image
from random import random, choice
from copy import copy, deepcopy

class Line():
    def __init__(self, p0, p1):
        self.p = np.array(p0)
        self.dirn = np.array(p1) - np.array(p0)
        self.dist = np.linalg.norm(self.dirn)
        self.dirn = self.dirn/ self.dist # normalize

    def path(self, t):
        return self.p + t * self.dirn
    
def Intersection(line, center, radius):
    a = np.dot(line.dirn, line.dirn)
    b = 2 * np.dot(line.dirn, line.p - center)
    c = np.dot(line.p - center, line.p - center) - radius * radius

    discriminant = b * b - 4 * a * c
    if discriminant < 0:
        return False

    t1 = (-b + np.sqrt(discriminant)) / (2 * a);
    t2 = (-b - np.sqrt(discriminant)) / (2 * a);

    if (t1 < 0 and t2 < 0) or (t1 > line.dist and t2 > line.dist):
        return False

    return True
    
    
def distance(x, y):
    return np.linalg.norm(np.array(x) - np.array(y))


def isInObstacle(vex, obstacles, radius):
    for obs in obstacles:
        if distance(obs, vex) < radius:
            return True
    return False


def isThruObstacle(line, obstacles, radius):
    for obs in obstacles:
        if Intersection(line, obs, radius):
            return True
    return False


def nearest(G, vex, obstacles, radius):
    Nvex = None
    Nidx = None
    minDist = float("inf")

    for idx, v in enumerate(G.vertices):
        line = Line(v, vex)
        if isThruObstacle(line, obstacles, radius):
            continue

        dist = distance(v, vex)
        if dist < minDist:
            minDist = dist
            Nidx = idx
            Nvex = v

    return Nvex, Nidx

class Graph:
    def __init__(self, startpos):
        self.startpos = startpos

        self.vertices = [startpos]
        self.edges = []
        self.success = False

        self.vex2idx = {startpos:0}
        self.neighbors = {0:[]}
        self.distances = {0:0.}


    def add_vex(self, pos):
        try:
            idx = self.vex2idx[pos]
        except:
            idx = len(self.vertices)
            self.vertices.append(pos)
            self.vex2idx[pos] = idx
            self.neighbors[idx] = []
        return idx

    def add_edge(self, idx1, idx2, cost):
        self.edges.append((idx1, idx2))
        self.neighbors[idx1].append((idx2, cost))
        self.neighbors[idx2].append((idx1, cost))


def RRT(startpos, obstacles, n_iter, radius, random_points):
    G = Graph(startpos)
    for it in range(n_iter):
        success = False
        trials = 0
        while not success:
            index = choice(range(len(random_points)))
            randvex = random_points[index]
            randvex = (randvex[0], randvex[1])
            trials += 1
            
            if trials > 300:
                print(trials)
                break
            if isInObstacle(randvex, obstacles, radius):
                continue

            nearvex, nearidx = nearest(G, randvex, obstacles, radius)
            if nearvex is None:
                continue
   
            newidx = G.add_vex(randvex)
            dist = distance(randvex, nearvex)
            G.add_edge(newidx, nearidx, dist)
            success = True
        if trials > 300:
            break
        random_points.pop(index)
        
        print('iter ' + str(it))
    return G

def remove_tuple_from_list(lst, tup):
    tup_dict = dict(lst)
    tup_dict.pop(tup)
    lst = list(tuple(tup_dict.items()))
    return lst

def depth_first_search(G):
    nodes = list(G.neighbors.keys())
    prev = {node: None for node in nodes}
    curNode = 0
    path = []
    nb = copy(G.neighbors)
    while len(nodes) > 1:
        path_i = []

        curNode = 0
        path_i.append(curNode)
        curNode = nb[curNode][0][0]
        path_i.append(curNode)

        if len(list(nb[curNode])) > 1:
            while True:
                curNode = nb[curNode][1][0]
                path_i.append(curNode)
                if len(list(nb[curNode])) <= 1:
                    break
        path_i_reverse = list(reversed(path_i))

        for i, node in enumerate(path_i_reverse[:-1]):
            if len(list(nb[node])) <= 1 and path_i_reverse[i]:
                child = path_i_reverse[i]
                parent = path_i_reverse[i+1]
                nodes.remove(child)
                del nb[child]
                nb[parent] = remove_tuple_from_list(nb[parent], child)

        path.append(path_i)
        path.append(path_i_reverse)
    return path
