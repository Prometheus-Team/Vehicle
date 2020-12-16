#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32, Int16, Header, Time, Empty, Int32
from sensor_msgs.msg import Range
from geometry_msgs.msg import Point
from vehicle_lib.msg import Distance, Speed, Steps, Location
from vehicle_lib.srv import ScanArea
from quaternion import Quaternion
import pyrr, urllib, json, time, math, threading


class VehicleAPI:
    URL = "http://192.168.0.101:8080/sensors.json"
    def __init__(self):
        self.seqC = 0
        self.inMotion = time.time()

        self.constSpeed = 1
        self.minFrontDistance = 20          # in centimeters, minimum allwed distance from obstacle

        self.heading = 0
        self.frontDistance = 0
        self.speedLeft = (0, 0)
        self.speedRight = (0, 0)
        self.steps = (0, 0)
        self.curLoc = Location(0,0,0,None)

        self.goalLoc = Location(0,0,0,None)


        # This topics will be used by the higher level API
        # ==================================================
        self.subFrontDistance = rospy.Subscriber('/pi/api/frontDistance', Distance, self.frontDistanceCallback)
        self.subHeading = rospy.Subscriber('/pi/api/heading', Float32, self.headingCallback)
        self.subSpeed = rospy.Subscriber('/pi/api/speed', Speed, self.speedCallback )
        self.subStep = rospy.Subscriber('/pi/api/step', Steps, self.stepsCallback )
        self.subCurLoc = rospy.Subscriber('/pi/localization/currentLoc', Steps, self.curLocCallback )

        self.pubSpeedCmd = rospy.Publisher('/pi/api/speedCmd', Speed, queue_size=10)
        # ==================================================
 

    def headingCallback(self, msg):
        self.heading = msg.data

    def frontDistanceCallback(self, msg):
        self.frontDistance = msg.distance

    def speedCallback(self, msg):
        self.inMotion = self.getCurTimeInMilliSecs()
        self.speedLeft = (msg.left, msg.expLeft)
        self.speedRight = (msg.right, msg.expRight)

    def stepsCallback(self, msg):
        self.steps = (msg.left, msg.right)

    def curLocCallback(self, msg):
        self.steps = (msg.x, msg.right)

    def getCurTimeInMilliSecs(self):
        return int(time.time() * 1000)

    def sendSpeedCmd(self, speedLeft, speedRight):    
        s = Speed(left=speedLeft, right=speedRight, header=self.getHeader())
        self.pubSpeedCmd.publish(s)

    # Service: Accept destination location to go
    def gotoService(self, msg):
        self.goalLoc = msg.end

        res = self.controlSpeed()

        return res[0]

    def computeDestinationAngle(self):
        if self.goalLoc.x - self.curLoc.x != 0:
            slope = (self.goalLoc.y - self.curLoc.y) / (self.goalLoc.x - self.curLoc.x)
            angle = (90 - math.degrees(math.atan(slope))) * math.copysign(slope, slope)

        else:
            angle = 90

        return angle

    def controlSpeed(self):
        reached = False
        goalOrientation = self.computeDestinationAngle()

        while not reached:
            angleDiff = goalOrientation - self.heading
            locDiff = self.calcDist(self.goalLoc, self.curLoc)

            if  locDiff <= 0.05 and abs(angleDiff) <= 5:
                return True, "Completed"

            if self.checkObstacle() > 0:
                return False, "Obstacle"

            if abs(angleDiff) > 20:
                if angleDiff > 0:
                    direc = "Right"
                else:
                    direc = "Left"
                
                self.turn(direc)
                time.sleep(0.5)
                continue

            if locDiff > 0.05:
                if abs(angleDiff) > 5 and angleDiff < 0:
                    rightFactor = 1.1

                elif abs(angleDiff) > 5 and angleDiff > 0:
                    leftFactor = 1.1

                self.driveStraight("Forward", leftFactor, rightFactor)
                time.sleep(0.3)


    def calcDist(self, pt1, pt2):
        return math.sqrt((pt1.x-pt2.x)**2 + (pt1.y-pt2.y)**2)


    def checkObstacle(self):
        if self.frontDistance < self.minFrontDistance:
            return self.frontDistance

        return 0

        # steer the vehicle with given angle in the given direction
    def turn(self, direction, factor=1):
        speedLeft = speedRight = self.constSpeed * factor

        if direction == 'Left':
            speedLeft *= -1

        elif direction == 'Right':
            speedRight *= -1

        else:
            speedLeft = speedRight = 0

        self.sendSpeedCmd(speedLeft, speedRight)

    # move dorward or backward with given speed
    def driveStraight(self, direction, leftFactor=1, rightFactor=1):
        speedLeft = speedRight = self.constSpeed
        if direction == 'Backward':
            speedLeft *= -1
            speedRight *= -1

        self.sendSpeedCmd(speedLeft, speedRight)

    def getHeader(self, seq=0):
        self.seqC += 1
        v = Header(self.seqC, None, '1')
        v.stamp = rospy.Time.now()
        return v

def main():
    r = rospy.init_node("VehicleAPI")
    # r.getParam('ip')
    VehicleAPI()
    rospy.spin()

if __name__=='__main__':
    main()
