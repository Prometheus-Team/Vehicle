#!/usr/bin/env python

import rospy, math
import numpy as np

from vehicle_lib.srv import GetShortestPath, GetShortestPathResponse
from vehicle_lib.msg import Location
from nav_msgs.msg import GridCells
from geometry_msgs.msg import Point


class Travel:
    A_STAR_STRGY = 'A*'
    def __init__(self):
        self.cellWidth = 0.3
        self.cellHeight = 0.3

        # self.map = np.zeros((20,20,20))
        self.parentPaths = {}
        self.edges = {}
        self.edgeCost = {}      # g in astar algorithm
        self.edgesIndex = {}
        self.randomMap()

        self.getShortestPathSrv = rospy.Service('/pi/travel/getShortestPath', GetShortestPath, self.getShortestPathCallback)
        self.pubMapViz = rospy.Publisher('/viz/map', GridCells, queue_size=10)

    def getShortestPathCallback(self, msg):
        start = msg.start
        end = msg.end
        grid = msg.map
        self.map = np.array([list(i.pts) for i in grid])
        print("Map total score: ",np.sum(self.map))
        # self.map = grid
        p = self.getShortestPath((start.x, start.y), (end.x, end.y))

        if p:
            path = self.getPathToFollow((start.x, start.y), p)
            path.reverse()

            pathN = self.simplifyPath(path)
            # pathN = path
            # print(pathN)
            # print(len(pathN))
            # print(len(path))
            if np.sum(self.map) > 0:
                # self.pubMapViz.publish(self.convertMapToPoints())

                return GetShortestPathResponse([Location(i[0], i[1],0,None) for i in pathN])

            return []

        return GetShortestPathResponse([])

    def randomMap(self):
        x = np.random.rand(1000,1000)
        x = x > 0.9
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
        cur = (int(cur[0]), int(cur[1]))
        # print(cur)
        for k in self.dimNodes(cur[0], sh[0]):
            if self.map[k, cur[1]] == 0:
                alts.append((k, cur[1]))

        for k in self.dimNodes(cur[1], sh[1]):
            if self.map[cur[0], k] == 0:
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
                # if l not in self.parentPaths:
                self.parentPaths[l] = self.edges[startEdge]
            

        return edges

    def aStar(self, start, end, edges):
        minCost = 100000
        next = edges[0][1]

        for edge in edges:
            # costG = self.edgeCost[edge]
            node = edge[1]
            # cost = self.hx(node, end) + costG
            cost = self.hx1(node, start, end)
            if cost < minCost:
                minCost = cost
                next = edge

        return next
    
    def hx(self, cur, end):
        return self.euclideanDist(cur, end)

    def hx1(self, cur, start, end):
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
        self.edgeCost[("Start", start)] = 0
        self.edgesIndex[0] = ("Start", start)

        frontier = []
        exploredEdges = set()
        exploredNodes = set()

        paths = self.getNextEdges(("Start", start), self.getNextNodes(start))

        for x in paths:
            if x not in exploredEdges:
                frontier.append(x)
                self.edgeCost[x] = self.edgeCost[("Start", start)] + 1


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
                    self.edgeCost[i] = self.edgeCost[pathToExpand] + 1



    # ngraphsearch
    # f - total score   g - cost behind     h - cost forward
    def getShortestPath1(self, start, end):
        self.edges[("Start", start)] = 0
        self.edgeCost[("Start", start)] = 0
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
        # print(finalEdge)
        path = [(finalEdge[1])]

        while finalEdge != ("Start", start):
            indFinalEdge = self.edges[finalEdge]
            path.append(finalEdge[0])
            finalEdge = self.edgesIndex[self.parentPaths[indFinalEdge]]
        return path

    def simplifyPath(self, path):
        pathN = [path[0]]
        slope = None
        prePoint = path[0]

        track = []
        for ptPre, pt in zip(path[:-1], path[1:]):
            if pt[0]-ptPre[0] == 0:
                slp = 'ver'

            else:
                slp = (pt[1] - ptPre[1])/(pt[0]-ptPre[0])

            if slope == None:
                slope = slp
                track.append((ptPre, slp))
                continue
            
            else:
                if str(slp) != 'ver' and str(slope) != 'ver' and math.atan(abs(slp-slope)) > math.radians(10):
                    print(slp, slope)
                    pathN.append(ptPre)
                    slope = slp
                    prePoint = pt
                    track.append((ptPre, slp))
                    if len(track)>3:
                        track.pop(0)

                    # slSt = track[0][1] 
                    # slL = track[-1][1] 
                    # if str(slp)!='ver' and str(slL) != 'ver' and str(slSt) != 'ver' and math.atan(abs(slL-slSt)) < math.radians(10):
                    #     pathN.pop()
                    #     pathN.pop()


                elif str(slp) != str(slope) and (str(slp) == 'ver' or str(slope) == 'ver'):
                    pathN.append(ptPre)
                    slope = slp
                    prePoint = pt
                    track.append((ptPre, slp))
                    if len(track)>3:
                        track.pop(0)

                    # slSt = track[0][1] 
                    # slL = track[-1][1] 
                    # if str(slp)!='ver' and str(slL) != 'ver' and str(slSt) != 'ver' and math.atan(abs(slL-slSt)) < math.radians(10):
                    #     pathN.pop()
                    #     pathN.pop()

                else:
                    track.append((ptPre, slp))
                    if len(track)>3:
                        track.pop(0)
                    continue

        return pathN

    def convertMapToPoints(self):
        points = GridCells()        
        pts = 0
        points.header.frame_id = "/my_frame"
        points.header.stamp = rospy.Time.now()
        points.cell_height = self.cellHeight
        points.cell_width = self.cellWidth

        map = self.map
        # map[0,0] = 1
        # map[0, self.height-1]=1
        # map[self.width-1,0]=1
        # map[self.width-1, self.height-1]=1

        for i in range(len(map)):
            for j in range(len(map[0])):
                if map[i, j]>0:
                    pts += 1
                    m = Point()
                    m.x = i * self.cellWidth
                    m.y = j * self.cellHeight

                    points.cells.append(m)

        return points




def main():
    rospy.init_node("ShortestPathComputingNode")
    a = Travel()
    # a.randomMap()
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