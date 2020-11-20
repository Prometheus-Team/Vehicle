import rospy
import numpy as np
import tf
import math
import geometry_msgs.msg
from geometry_msgs.msg import Point
from nav_msgs.msg import GridCells
from visualization_msgs.msg import Marker, MarkerArray
import random

class MapViz:
    def __init__(self, x=20, y=20, cellWidth=0.3, cellHeight=0.3):
        self.x = x
        self.y = y
        self.cellWidth = 0.3
        self.cellHeight = 0.3

        # rospy.init_node("MapViz")
        self.rate = rospy.Rate(1)

        self.pubMapViz = rospy.Publisher('/viz/map', GridCells, queue_size=10)
        self.pubPathViz = rospy.Publisher('/viz/path', GridCells, queue_size=10)
        self.pubCurLocViz = rospy.Publisher('/viz/curLoc', GridCells, queue_size=10)

        self.subMap = rospy.Subscriber('/pi/mapper/map', , self.readSpeedLeft)
        self.subPath = rospy.Subscriber('/pi/mapper/path', Float32, self.readSpeedLeft)
        self.subCurLoc = rospy.Subscriber('/pi/localization/curLoc', Float32, self.readSpeedLeft)


    def sampleVoxels(self):
        return np.random.rand(self.x, self.y) < 0.5

    def samplePath(self):
        path = np.random.rand(self.x, self.y) < 0.2
        pts = []

        for i in range(self.x):
            for j in range(self.y):
                if path[i, j]>0:
                    pts.append((i, j))

        return pts

    def visualizeMap(self, grid):
        points = GridCells()        
        pts = 0
        points.header.frame_id = "/my_frame"
        points.header.stamp = rospy.Time.now()
        points.cell_height = self.cellHeight
        points.cell_width = self.cellWidth

        for i in range(self.x):
            for j in range(self.y):
                if grid[i, j]>0:
                    pts += 1
                    m = Point()
                    m.x = i * self.cellWidth
                    m.y = j * self.cellHeight

                    points.cells.append(m)

        return points

    def visualizePath(self, path):
        points = GridCells()        
        pts = 0
        points.header.frame_id = "/my_frame"
        points.header.stamp = rospy.Time.now()
        points.cell_height = self.cellHeight
        points.cell_width = self.cellWidth

        for pt in path:
            pts += 1
            m = Point()
            m.x = pt[0] * self.cellWidth
            m.y = pt[1] * self.cellHeight

            points.cells.append(m)

        return points

    def run(self, grid=None, path=None):
        # grid = self.sampleVoxels()
        # path = self.samplePath()
        c = 0
        while not rospy.is_shutdown():
            # if c>1:
            #     return
            # c += 1

            if grid is not None:
                mk = self.visualizeMap(grid)
                self.pubMap.publish(mk)
    
            if path is not None:
                mk1 = self.visualizePath(path)
                self.pubPath.publish(mk1)
            
            self.rate.sleep()  
                  

def main():
    rospy.init_node("VisualizeMap")
    MapViz()
    rospy.spin()

if __name__=='__main__':
    main()

# if __name__== '__main__':
#     print("2D grid being visualized")
#     MapViz().run()
