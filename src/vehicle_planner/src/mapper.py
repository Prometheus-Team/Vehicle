#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Range
from std_msgs.msg import Empty, Float32
from geometry_msgs.msg import Point
from nav_msgs.msg import GridCells

from vehicle_lib.msg import Distance, Location, IntArr
from vehicle_lib.srv import GetMap, InitMap, GetShortestPath, GetMapResponse, InitMapResponse
from tf import transformations as tra

import math, time
import numpy as np
# from visualizeMap import MapViz

class MapBuilder:
    def __init__(self, width=1000, height=1000):
        self.cellWidth = 0.3
        self.cellHeight = 0.3
        self.map = None
        self.heading = 0
        self.curLoc = (0, 0)
        self.lastScanTime = 0
        self.scanCompleted = True

        self.maxCorrectDistance = 300
        self.bufferLayer = 7
        self.worldToMapRatio = 0.02    # 1 cell = 0.02 meters in the world 
        self.extraBeyondBorder = 100  # in grid cell count

        self.width = width
        self.height = height
        self.curLocC = 0


        self.pubScanArea = rospy.Publisher('/pi/api/scanCmd', Empty, queue_size=10)
        
        self.subRange = rospy.Subscriber('/pi/api/range', Distance, self.scanCallback)
        self.subHeading = rospy.Subscriber('/pi/api/heading', Float32, self.headingListener)
        self.subCurLoc = rospy.Subscriber('/pi/localization/curLoc', Location, self.curLocListener)

        self.pubMapViz = rospy.Publisher('/viz/map', GridCells, queue_size=10)
        self.pubPathViz = rospy.Publisher('/viz/path', GridCells, queue_size=10)
        self.pubCurLocViz = rospy.Publisher('/viz/curLoc', GridCells, queue_size=10)

        # Initialize the map and return it when requested 
        self.initMapSrv = rospy.Service('/pi/mapper/initMap', InitMap, self.initMapCallback)
        self.getMapSrv = rospy.Service('/pi/mapper/getMap', GetMap, self.getMapCallback)

        self.getShortestPathSrv = rospy.ServiceProxy('/pi/travel/getShortestPath', GetShortestPath)

        self.initMap(self.width, self.height)

    def initMap(self, x, y, layer=11):
        self.map = np.zeros((int(layer), int(x), int(y)))

    def initMapCallback(self, msg):
        try:
            self.initMap(msg.width+self.extraBeyondBorder, msg.height+self.extraBeyondBorder)
            return InitMapResponse(True)

        except:
            return InitMapResponse(False)

    def getMap(self):
        return self.map[0:10,:,:].sum(axis=0) > 0

    def getMapCallback(self, msg):
        try:
            for i in range(3):
                if not self.scanCompleted:
                    time.sleep(3)
                
                elif not self.scanCompleted and i >= 2:
                    return GetMapResponse([])

                else:
                    m = self.getMap()
                    return GetMapResponse(self.convertMapToArr(m))

        except:
            return GetMapResponse([])

    def curLocListener(self, msg):
        self.setCurLoc(msg.data)

    def getCurLoc(self):
        return self.curLoc

    def setCurLoc(self, curLoc):
        self.curLoc = curLoc

    def headingListener(self, msg):
        self.setHeading(msg.data)

    def getHeading(self):
        return self.heading

    def setHeading(self, heading):
        self.heading = heading

    def scanCallback(self, msg):
        self.scanCompleted = False
        curLocs = [(0, 0), (500,300), (-400,800),(-300,-600),(400,-600)]
        curLoc = curLocs[self.curLocC%5]
        dirs = [0, 30, 45, -60, -120]
        dir = dirs[self.curLocC%5]
        dests = [(200,350),(200,350),(200,350),(200,350),(200,350)]
        dest = dests[self.curLocC%5]
        # curLoc = self.curLoc
        # dir = self.heading
        i = 180-msg.angle

        if msg.distance <= self.maxCorrectDistance:
            if i>10 and i<170:
                j = 0
                # for j in range(-10, 11):
                pt = self.convertPointToGrid(self.convertRangeToGlobalPoint(msg.distance, i+j, curLoc, dir))
                self.plotPointOnMap(pt)

            else:
                pt = self.convertPointToGrid(self.convertRangeToGlobalPoint(msg.distance, i, curLoc, dir))
                self.plotPointOnMap(pt)

        if msg.last:
            print("Visualizing.....")
            self.scanCompleted = True
            self.curLocC += 1

            self.pubMapViz.publish(self.convertMapToPoints())
            # self.pubPathViz.publish(self.convertCurLocToPoint(gridPath))
            self.pubCurLocViz.publish(self.convertCurLocToPoint([self.convertPointToGrid(curLoc)]))

        self.lastScanTime = time.time()

    def degToRad(self, ang):
        return ang * math.pi / 180

    def radToDeg(self, ang):
        return ang * 180 / math.pi

    def convertRangeToPoint(self, distance, angle):
        angleN = self.degToRad(angle)

        pt = self.transform((distance,0), angleN)

        return round(pt[0], 3), round(pt[1], 3)

    def convertRangeToGlobalPoint(self, distance, angle, curLoc, heading=0):
        angleN = self.degToRad(angle)

        pt = self.transform((distance,0), angleN)

        point = pt[:2]

        angleN = self.degToRad(heading)

        pt = self.transform(point, angleN, curLoc)

        return round(pt[0], 3), round(pt[1], 3)


    def convertPointToGrid(self, pt):
        return math.floor((self.width/2) + pt[0]), math.floor((self.height/2) + pt[1])

    def plotPointOnMap(self, point):
        if point[0]>0 and point[1]>0 and point[0] < self.width and point[1] < self.height:
            layer = self.map[10, int(point[1]), int(point[0])]
            if layer < 10:
                x = int(point[1])
                y = int(point[0])
                self.map[int(layer), x, y] = 1
                self.plotBufferOnMap(x, y, layer)
                self.map[10, int(point[1]), int(point[0])] += 1

    def plotBufferOnMap(self, x, y, layer):
        buf = self.getBuffer(x, y)
        for i in buf:
            if i[0] < self.width and i[1] < self.height:
                self.map[int(layer), i[0], i[1]] = 2

    def getBuffer(self, x, y):
        layer = self.bufferLayer
        a = []
        for k in range(layer):
            l = k+1
            for i in range(-l, l+1):
                for j in [l, -l]:
                    a.append((x + i, y + j))
                    a.append((x + j, y + i))

        return set(a)

    # point - the point to be transformed
    # traVec - translation vector
    # rotAng - rotation angle to rotate the point around the z-axis
    def transform(self, point, rotAng, traVec=(0,0)):
        point = (point[0], point[1], 0)
        traVec = (traVec[0], traVec[1], 0)
        mt = tra.translation_matrix(traVec)
        mr = tra.rotation_matrix(rotAng, (0, 0, 1))
        mat = mt.dot(mr)    # rotate then translate
        # mat = mr.dot(mt)  # translate then rotate
        point = [point[0], point[1], 0, 1]

        return tuple(mat.dot(point)[:2])

    def convertMapToPoints(self):
        points = GridCells()        
        pts = 0
        points.header.frame_id = "/my_frame"
        points.header.stamp = rospy.Time.now()
        points.cell_height = self.cellHeight
        points.cell_width = self.cellWidth

        map = self.getMap()
        map[0,0] = 1
        map[0, self.height-1]=1
        map[self.width-1,0]=1
        map[self.width-1, self.height-1]=1

        for i in range(len(map)):
            for j in range(len(map[0])):
                if map[i, j]>0:
                    pts += 1
                    m = Point()
                    m.x = i * self.cellWidth
                    m.y = j * self.cellHeight

                    points.cells.append(m)

        return points

    def convertCurLocToPoint(self, curLocs):
        points = GridCells()        
        pts = 0
        points.header.frame_id = "/my_frame"
        points.header.stamp = rospy.Time.now()
        points.cell_height = self.cellHeight
        points.cell_width = self.cellWidth

        for loc in curLocs:
            pts += 1
            m = Point()
            m.x = loc[0] * self.cellWidth
            m.y = loc[1] * self.cellHeight

            points.cells.append(m)

        return points

    def convertMapToArr(self, map):
        return [IntArr(i) for i in map]
        # arr = []
        # for i in self.map:
        #     arr.append(IntArr(i))

        # return arr

    # def testRange(self):
    #     import random
    #     firstLoc = [(0,0),(150,50)]
    #     dirs = [0,-90]
    #     curLocs = []
    #     f = 0
    #     for l in firstLoc:
    #         curLocs.append(self.convertPointToGrid(l, l))
    #         for i in range(0, 181, 2):
    #             if (i>=0 and i<=45) or (i>=135 and i<=180):
    #                 # x = random.randint(65,75)
    #                 x = random.randint(15,25)
    #             else:
    #                 # continue
    #                 x = random.randint(30, 50)

    #             if i >= 180:
    #                 dis = Distance(i, x, True)
    #             else:
    #                 dis = Distance(i, x, False)


    #             # # print(x, i)
    #             # if i>10 and i<170:
    #             #     for j in range(-10, 11):
    #             #         pt = self.convertPointToGrid(l, self.convertRangeToPoint(x, i+j), dirs[f])
    #             #         self.plotPointOnMap(pt)

    #             # else:
    #             #     pt = self.convertPointToGrid(l, self.convertRangeToPoint(x, i), dirs[f])
    #             #     self.plotPointOnMap(pt)
            
    #             # print(self.map[int(pt[1]), int(pt[0])])
    #         f += 1
    #     print(np.sum(self.getMap()))
    #     # while True:
    #     # pt = self.convertPointToGrid((0,0), (0,0))
    #     # self.viz.run(self.getMap(), curLocs)


def main():
    rospy.init_node("MapperNode")
    MapBuilder(1000,1000)
    rospy.spin()

if __name__=='__main__':
    main()

# x = MapBuilder(400,400)
# # print(x.convertRangeToPoint(70, 180))
# # print(x.transform((2,4),(1,1,0),math.pi/2))
# # print(x.transform((2,4),(0,0,0),math.pi/2))
# print(x.testRange())


        # if angleN > math.pi/2:
        #     return round(-math.cos((math.pi) - angleN) * distance, 3), round(math.sin((math.pi)-angleN) * distance, 3)
        
        # return round(math.cos(angleN) * distance, 3), round(math.sin(angleN) * distance, 3)
