#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Range
from std_msgs.msg import Empty, Float32
from geometry_msgs.msg import Point
from nav_msgs.msg import GridCells

from vehicle_lib.msg import Distance, Location, IntArr
from vehicle_lib.srv import GetMap, InitMap
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

        self.width = width
        self.height = height


        self.pubScanArea = rospy.Publisher('/pi/api/scanCmd', Empty, queue_size=10)
        
        self.subRange = rospy.Subscriber('/pi/api/range', Distance, self.scanCallback)
        self.subHeading = rospy.Subscriber('/pi/api/heading', Float32, self.headingListener)
        self.subCurLoc = rospy.Subscriber('/pi/localization/curLoc', Location, self.curLocListener)

        self.pubMapViz = rospy.Publisher('/viz/map', GridCells, queue_size=10)
        self.pubPathViz = rospy.Publisher('/viz/path', GridCells, queue_size=10)
        self.pubCurLocViz = rospy.Publisher('/viz/curLoc', GridCells, queue_size=10)

        # Initialize the map and return it when requested 
        self.initMapSrv = rospy.Service('/pi/mapper/initMap', InitMap, self.getMapHandler)
        self.getMapSrv = rospy.Service('/pi/mapper/getMap', GetMap, self.getMapHandler)


        self.initMap(self.width, self.height)

    def initMap(self, x, y, layer=11):
        self.map = np.zeros((int(layer), int(x), int(y)))

    def initMapHandler(self, msg):
        try:
            self.initMap(msg.width, msg.height)
            return True

        except:
            return False

    def getMap(self):
        return self.map[0:10,:,:].sum(axis=0) > 0

    def getMapHandler(self, msg):
        try:
            for i in range(3):
                if not self.scanCompleted:
                    time.sleep(3)
                
                elif not self.scanCompleted and i >= 2:
                    return []

                else:
                    m = self.getMap()
                    return self.convertMapToArr(m)

        except:
            return []

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
        curLoc = (0, 0)
        dir = 0
        i = 180-msg.angle

        if msg.distance <= self.maxCorrectDistance:
            if i>10 and i<170:
                for j in range(-10, 11):
                    pt = self.convertPointToGrid(curLoc, self.convertRangeToPoint(msg.distance, i+j), dir)
                    self.plotPointOnMap(pt)

            else:
                pt = self.convertPointToGrid(curLoc, self.convertRangeToPoint(msg.distance, i), dir)
                self.plotPointOnMap(pt)

        if msg.last:
            self.scanCompleted = True
            self.pubMapViz.publish(self.convertMapToPoints())
            self.pubCurLocViz.publish()

        self.lastScanTime = time.time()

    def degToRad(self, ang):
        return ang * math.pi / 180

    def radToDeg(self, ang):
        return ang * 180 / math.pi

    def convertRangeToPoint(self, distance, angle):
        angleN = self.degToRad(angle)

        pt = self.transform((distance,0), angleN)

        return round(pt[0], 3), round(pt[1], 3)

        # if angleN > math.pi/2:
        #     return round(-math.cos((math.pi) - angleN) * distance, 3), round(math.sin((math.pi)-angleN) * distance, 3)
        
        # return round(math.cos(angleN) * distance, 3), round(math.sin(angleN) * distance, 3)

    def convertPointToGrid(self, curLoc, point, heading=0):
        angleN = self.degToRad(heading)

        pt = self.transform(curLoc, point, angleN)
        
        return math.floor((self.width/2) + pt[0]), math.floor((self.height/2) + pt[1])

    def plotPointOnMap(self, point):
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
        mr = tra.rotation_matrix(rotAng, (0, 0, 1),(0,0,0))
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
        for i in range(len(map)):
            for j in range(len(map[0])):
                if map[i, j]>0:
                    pts += 1
                    m = Point()
                    m.x = i * self.cellWidth
                    m.y = j * self.cellHeight

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