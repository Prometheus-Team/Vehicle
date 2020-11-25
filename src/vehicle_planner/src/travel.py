#!/usr/bin/env python

import rospy, math
import numpy as np

from vehicle_lib.srv import GetShortestPath
from vehicle_lib.msg import Location

class Travel:
    A_STAR_STRGY = 'A*'
    def __init__(self):
        # self.map = np.zeros((20,20,20))
        self.parentPaths = {}
        self.edges = {}
        self.edgesIndex = {}
        self.randomMap()

        self.getShortestPathSrv = rospy.Service('/pi/travel/getShortestPath', GetShortestPath, self.getShortestPathCallback)

    def getShortestPathCallback(self, msg):
        start = msg.start
        end = msg.end
        grid = msg.map
        self.map = np.array([list(i.pts) for i in grid])

        p = self.getShortestPath((start.x, start.y), (end.x, end.y))

        if p:
            path = self.getPathToFollow((start.x, start.y), p)
            path.reverse()

            return [Location(i[0], i[1]) for i in path]

        return []

    def randomMap(self):
        x = np.random.rand(20,20)
        x = x > 0.7
        x = x.astype(int)
        self.map = x

    def dimNodes(self, val, size):
        vals = []

        # vals.append(val)
        if val + 1 < size:
            vals.append(val+1)

        if val - 1 >= 0:
            vals.append(val-1)

        return vals

    def getNextNodes(self, cur):
        sh = self.map.shape
        alts = []

        for k in self.dimNodes(cur[0], sh[0]):
            alts.append((k, cur[1]))

        for k in self.dimNodes(cur[1], sh[1]):
            alts.append((cur[0], k))

        return alts

    def getNextEdges(self, startEdge, endNodes):
        startNode = startEdge[1]
        edges = []
        for i in endNodes:
            edge = (startNode, i)
            edges.append(edge)
            l = len(self.edges)
            if edge not in self.edges:
                self.edges[edge] = l
                self.edgesIndex[l] = edge
                if l not in self.parentPaths:
                    self.parentPaths[l] = self.edges[startEdge]
            

        return edges

    def aStar(self, start, end, edges):
        minCost = 100000
        next = edges[0][1]

        for edge in edges:
            node = edge[1]
            cost = self.hx(node, start, end)
            if cost < minCost:
                minCost = cost
                next = edge

        return next
    
    def hx(self, cur, start, end):
        return self.euclideanDist(cur, start) + self.euclideanDist(cur, end)

    def euclideanDist(self, s, e):
        return math.sqrt((e[0]-s[0])**2 + (e[1]-s[1])**2)

    # def search(self, start, end):
    #     reached = False
    #     cur = start

    #     while not reached:
    #         self.explored.add(cur)
    #         nodes = self.getNextNodes(cur)
    #         cur = self.aStar(start, end, nodes)

    def pathToExpand(self, strategy, frontier, start, end):
        if strategy == self.A_STAR_STRGY:
            return self.aStar(start, end, frontier)

        return None

    # ngraphsearch
    def getShortestPath(self, start, end):
        self.edges[("Start", start)] = 0
        self.edgesIndex[0] = ("Start", start)

        frontier = []
        exploredEdges = set()
        exploredNodes = set()

        paths = self.getNextEdges(("Start", start), self.getNextNodes(start))

        for x in paths:
            if x not in exploredEdges:
                frontier.append(x)

        while True:
            if not frontier:
                return False

            pathToExpand = self.pathToExpand(self.A_STAR_STRGY, frontier, start, end)
            nodeToExpand = pathToExpand[1]
            frontier.remove(pathToExpand)

            if pathToExpand not in exploredEdges and nodeToExpand not in exploredNodes:
                exploredEdges.add(pathToExpand)
            else:
                continue

            if pathToExpand[1] not in exploredNodes:
                exploredNodes.add(nodeToExpand)

            # Goal Test
            if nodeToExpand == end:
                return pathToExpand

            for i in self.getNextEdges(pathToExpand, self.getNextNodes(nodeToExpand)):
                if i not in exploredEdges and i not in frontier and i[1] not in exploredNodes:
                    frontier.append(i)


    def getPathToFollow(self, start, finalEdge):
        print(finalEdge)
        path = [(finalEdge[1])]

        while finalEdge != ("Start", start):
            indFinalEdge = self.edges[finalEdge]
            path.append(finalEdge[0])
            finalEdge = self.edgesIndex[self.parentPaths[indFinalEdge]]
        return path


def main():
    rospy.init_node("ShortestPathComputingNode")
    Travel()
    rospy.spin()

if __name__=='__main__':
    main()






# from visualizeMap import MapViz
# from pathSmoother import SmoothPath

# def main():
#     srch = Travel()
#     map = MapViz()
#     sm = SmoothPath()

#     strt = (2,3)
#     x = srch.getShortestPath(strt,(13,16),srch.A_STAR_STRGY)

#     if x:
#         path = srch.getPathToFollow(strt, x)
#         path.reverse()
#         print(path)
#         c = sm.smoothen(path)
#         sm.printPaths(path, c)
#         map.run(srch.map, c)

#         return

#     print("No path found")
#     return

    

# main()
# srch.search()