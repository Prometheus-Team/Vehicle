#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32, Int16
from sensor_msgs.msg import Range
from vehicle_lib.msg import Distance, Speed, Location, Steps
from quaternion import Quaternion
import pyrr, urllib, json, time, math, threading

import numpy as np
import enum
import pickle

class Direction(enum.Enum):
    Stationary = 0
    Forward = 1
    Backward = 2
    Left = 3
    Right = 4

class TravelData:
    def __init__(self, timestamp=None, heading=0, range=0, speedLeft=None, speedRight=None, steps=None, imu=None):
        self.timestamp = timestamp
        self.heading = heading
        self.range = range      # distance from front obstacle
        self.speedLeft = speedLeft      # (speed, expSpeed) direction can be 0 - Forward  1 - Backward
        self.speedRight = speedRight      
        self.imu = imu
        self.steps = steps      # (left, right)
        self.leftAcceleration = 0
        self.rightAcceleration = 0  
        self.direction = Direction.Forward

        # self.modifySpeed()

    def modifySpeed(self):
        if self.speedLeft:
            self.speedLeft = (self.speedLeft[0]/60.0, self.speedLeft[1])

        if self.speedRight:
            self.speedRight = (self.speedRight[0]/60.0, self.speedRight[1])

    def getDiff(self, preData):
        diff = TravelData()
        diff.timestamp = self.timestamp - preData.timestamp
        diff.heading = self.heading - preData.heading
        # diff.range = self.range - preData.range        
        diff.steps = (self.steps[0]-preData.steps[1], self.steps[1]-preData.steps[1])

        acc = self.computeAccelration(preData)
        diff.leftAcceleration = acc[0]
        diff.rightAcceleration = acc[1]

        diff.direction = self.computeDirection(preData)

        return diff

    def computeAccelration(self, preData):
        rightAcc = (self.speedRight[0] - preData.speedRight[0])/((self.timestamp - preData.timestamp)/1000.0)
        leftAcc = (self.speedLeft[0] - preData.speedLeft[0])/((self.timestamp - preData.timestamp)/1000.0)
        
        return leftAcc, rightAcc 

    def computeDirection(self, preData):
        if self.speedLeft[1] == 0 and self.speedRight[1] == 0:
            return Direction.Stationary

        elif self.speedLeft[1] > 0 and self.speedRight[1] > 0:
            if self.speedLeft[1] == self.speedRight[1]:
                return Direction.Forward

            elif self.speedLeft[1] > self.speedRight[1]:
                return Direction.Right

            elif self.speedLeft[1] < self.speedRight[1]:
                return Direction.Left

        elif self.speedLeft[1] < 0 and self.speedRight[1] < 0:
            return Direction.Backward

        return None


class LastLocation:
    def __init__(self, x, y, heading, timestamp):
        self.x = x
        self.y = y
        self.heading = heading
        self.timestamp = timestamp

class Prediction:
    def __init__(self, speedDis, stepDis, rangeDis, heading):
        self.speedEff = 0.7
        self.stepEff = 0.4
        self.rangeEff = 0.5

        self.speedDis = speedDis
        self.stepDis = stepDis
        self.rangeDis = rangeDis
        self.imuDis = None
        self.heading = heading

    def combine(self):
        avg = (self.speedDis + self.stepDis + self.rangeDis) / 3

        return avg

    def getCoord(self, distance):
        xSpeed = math.sin(self.degToRad(self.heading)) * distance
        ySpeed = math.cos(self.degToRad(self.heading)) * distance

        xStep = math.sin(self.degToRad(self.heading)) * distance
        yStep = math.cos(self.degToRad(self.heading)) * distance
        
        if self.rangeDis < 5:
            xRange = math.sin(self.degToRad(self.heading)) * distance
            yRange = math.cos(self.degToRad(self.heading)) * distance

            return ((xSpeed, ySpeed), (xStep, yStep), (xRange, yRange))

        return ((xSpeed, ySpeed), (xStep, yStep), None)

    def degToRad(self, ang):
        return ang * math.pi / 180

    def radToDeg(self, ang):
        return ang * 180 / math.pi

class Localization:
    def __init__(self):
        self.dataRecordFreq = 50        # number of times we record data per second
        self.maxValidTime = 50
        self.wheelRadius = 0.033
        self.wheelGap = 0.14    # in meters
        self.revToStepRatio = 20    # 1 rev = 20 steps
        self.distanceDiffTolerance = 0.05   # in meters
        self.headingDiffTolerance = 2   # in angles
        self.rot = 0

        self.lastKnownLocation = LastLocation(0,0,0,time.time())
        self.inMotion = 0
        self.saved = False

        self.heading = 0
        self.range = 0
        self.speedLeft = (0, 0)
        self.speedRight = (0, 0)
        self.steps = (0, 0)
        self.imu = None

        self.travelTrace = {}
        self.travelPathTime = []
        self.speedTrace = {}  # (time: (speedLeft, speedRight))

        dataRecording = threading.Thread(target=self.recordData)

        self.pubCurLoc = rospy.Publisher('/pi/localization/currentLoc', Location, queue_size=10)

        self.subFrontDistance = rospy.Subscriber('/pi/api/frontDistance', Distance, self.frontDistanceCallback)
        self.subHeading = rospy.Subscriber('/pi/api/heading', Float32, self.headingCallback)
        self.subSpeed = rospy.Subscriber('/pi/api/speed', Speed, self.speedCallback )
        self.subStep = rospy.Subscriber('/pi/api/step', Steps, self.stepsCallback )


        dataRecording.daemon = True
        dataRecording.start()
        dataRecording.join()

    def recordData(self):
        while True:
            # print("Diff",self.getCurTimeInMilliSecs() ,self.inMotion,self.getCurTimeInMilliSecs() - self.inMotion)
            if self.getCurTimeInMilliSecs() - self.inMotion < 200:
                self.saved = False
                # print("Record")
                t = self.getCurTimeInMilliSecs()
                s = self.getSteps()
                tra = TravelData(t, self.getHeading(), self.getRange(), self.getSpeedLeft(), self.getSpeedRight(), s, self.getIMU())
                self.travelTrace[t] = tra
                self.speedTrace[t] = s
                # print(s)

                time.sleep(1/self.dataRecordFreq)

            # elif len(self.travelTrace) > 0 and self.saved is False:
            elif len(self.travelTrace) > 0 and self.saved is False:
                # print("Save to file")
                # self.save()
                self.saved = True
                # return
                # self.computeSpeed()
                # print("Compute", len(self.travelTrace))
                self.estimateLocation(self.lastKnownLocation.timestamp, self.inMotion)

    def computeSpeed(self):
        ts = self.speedTrace.keys()
        ts.sort()
        for i,j in zip(ts[:-1],ts[1:]):
            print(i,j)
            leftPre = self.speedTrace.get(i)[0]
            rightPre = self.speedTrace.get(i)[1]

            leftPost = self.speedTrace.get(j)[0]
            rightPost = self.speedTrace.get(j)[1]

            l = ((leftPost - leftPre)/20.0) * math.pi * 2 * 0.033
            r = ((rightPost - rightPre)/20.0) * math.pi * 2 * 0.033

            print(l, r)

    def headingCallback(self, msg):
        self.heading = msg.data
        if self.lastKnownLocation.timestamp == 0:
            self.lastKnownLocation.heading = self.heading
            self.lastKnownLocation.timestamp = self.getCurTimeInMilliSecs()

    def frontDistanceCallback(self, msg):
        self.range = msg.distance

    def speedCallback(self, msg):
        # print("Speed",self.speedLeft)
        self.rot += 1

        self.inMotion = self.getCurTimeInMilliSecs()
        self.speedLeft = (msg.left, msg.expLeft)
        self.speedRight = (msg.right, msg.expRight)

        if self.lastKnownLocation.timestamp == 0:
            self.lastKnownLocation.heading = self.heading
            self.lastKnownLocation.timestamp = self.getCurTimeInMilliSecs()

    def stepsCallback(self, msg):
        self.steps = (msg.left, msg.right)

    def getHeading(self):
        return self.heading

    def getRange(self):
        return self.range

    def getSpeedLeft(self):
        return self.speedLeft

    def getSpeedRight(self):
        return self.speedRight

    def getSteps(self):
        return self.steps

    def getIMU(self):
        return self.imu

    def getTotalSpeed(self, speed, heading):
        return None

    def getCurTimeInMilliSecs(self):
        return int(time.time() * 1000)
        # return time.time()

    def getTravelData(self, startTime, endTime):
        # print(self.travelTrace)
        times = np.array(self.travelTrace.keys())
        times = times[(times>=startTime) & (times<endTime)]
        times.sort()
        return times

    def manipulateData(self, times):
        v = {}
        preSpeedLeft = preSpeedRight = 0

        for i in times:
            a = self.travelTrace.get(i)
            if a.speedLeft != preSpeedLeft or a.speedRight != preSpeedRight:
                v[i] = a

        return v


    def estimateLocation(self, startTime, endTime):
        import sys
        t = 0
        preds = []
        times = self.getTravelData(startTime, endTime)
        v = self.manipulateData(times)
        ts = v.keys()
        ts.sort()
        # ts = times
        print("here1")
        st = None

        for tStart, tEnd in zip(ts[:-1], ts[1:]):
            # tStart = tStart/1000
            # tEnd = tEnd/1000
            # print(tEnd - tStart)
            if tEnd - tStart <= self.maxValidTime:
                dataStart = self.travelTrace.get(tStart)
                dataEnd = self.travelTrace.get(tEnd)
                t += dataStart.speedRight[0]
                # print(dataStart.steps)
                # print(dataEnd.range)
                diff = dataEnd.getDiff(dataStart)

                # distance from speedometer
                leftDisSpeed = self.computeDistanceFromAccel(dataStart.speedLeft[0], diff.leftAcceleration, diff.timestamp/1000.0)
                rightDisSpeed = self.computeDistanceFromAccel(dataStart.speedRight[0], diff.rightAcceleration, diff.timestamp/1000.0)
                # print(diff.leftAcceleration, diff.rightAcceleration)
                speedDis = self.combineDistance(leftDisSpeed, rightDisSpeed, diff.heading)
                # print(leftDisSpeed, rightDisSpeed, speedDis)

                # distance from steps
                leftDisStep = self.computeDistanceFromSteps(diff.steps[0])
                rightDisStep = self.computeDistanceFromSteps(diff.steps[1])
                stepDis = self.combineDistance(leftDisStep, rightDisStep, diff.heading)
                # print(leftDisStep)
                pred = Prediction(speedDis, stepDis, diff.range, diff.heading)
                preds.append(pred)


        if len(preds) > 0:
            x, y, x1, y1 = self.combineLoc(preds)
            stS = self.travelTrace.get(times[0]).steps
            stL = self.travelTrace.get(times[-1]).steps
            cir = 2*math.pi*self.wheelRadius
            d = ((((stL[0] - stS[0])/20.0)*cir) + (((stL[1] - stS[1])/20.0)*cir))/2.0
            # # d = (((stL[1] - stS[1]-20)/20.0)*0.207)
            print(stL[0]- stS[0], stL[1]- stS[1])
            print(x,y,x1,y1)
            # print(self.getCurrentLoc(x, y))
            # print("Here ",((float(t)/len(ts)) + y)/2, len(ts), startTime-endTime)
            self.travelTrace = {}
            # self.rot = 0
            # k = (stL[1] - stS[1])/20.0
            print("Distance Exact: ", d, ((stL[0] - stS[0]) + (stL[1] - stS[1]))/200.0)


            # self.lastKnownLocation.x = x
            # self.lastKnownLocation.y = y
            # self.lastKnownLocation.timestamp = self.inMotion

            # sys.exit()

    def combineLoc(self, preds):
        xSp=xSt = 0
        ySp=ySt = 0

        for pred in preds:
            coordSp = pred.getCoord(pred.speedDis)
            coordSt = pred.getCoord(pred.stepDis)
            # coord = pred.getCoord(pred.stepDis)
            xSp += coordSp[0][0]
            ySp += coordSp[0][1]
            xSt += coordSt[1][0]
            ySt += coordSt[1][1]

        return xSp, ySp, xSt, ySt

    def getCurrentLoc(self, x, y):
        return self.lastKnownLocation.x + x, self.lastKnownLocation.y + y

    def computeDistanceFromAccel(self, vi, acc, t):
        return vi * t + (0.5 * acc * t**2)

    def computeDistanceFromSteps(self, stepDiff):
        return 2 * math.pi * self.wheelRadius * (stepDiff/self.revToStepRatio)

    def combineDistance(self, left, right, angle):
        # print(left-right)
        if angle < self.headingDiffTolerance or abs(left-right) <= self.distanceDiffTolerance:
            return (left+right)/2

        elif left < right and angle < 0:
            rIn = left/abs(angle)
            rOut = (right/abs(angle)) - self.wheelGap

        elif left > right and angle > 0:
            rIn = right/angle
            rOut = (left/angle) - self.wheelGap

        cIn = 2 * (rIn + (self.wheelGap)/2) * math.sin(self.degToRad(angle)/2)
        cOut = 2 * (rOut - (self.wheelGap/2)) * math.sin(self.degToRad(angle)/2)

        return (cIn + cOut) / 2

    def degToRad(self, ang):
        return ang * math.pi / 180

    def radToDeg(self, ang):
        return ang * 180 / math.pi

    def currentLoc(self):
        startTime = self.lastKnownLocation.timestamp
        endTime = self.getCurTimeInMilliSecs()
        
        # rangeLoc = self.estimateLocationFromRange(startTime, endTime)
        # speedLoc = self.estimateLocationFromSpeedometer(startTime, endTime)
        # imuLoc = self.estimateLocationFromIMU(startTime, endTime)


    def computeResultantLoc(self, rangeLoc, speedLoc, imuLoc):
        return None

    def save(self):
        f = open('travelTrace', 'wb')
        pickle.dump(self.travelTrace, f)
        f.close()

    def read(self):
        f = open('travelTrace', 'rb')
        a = pickle.load(f)
        f.close()

        return a

    def test(self):
        print("Called")
        self.travelTrace = self.read()
        a = self.travelTrace.keys()
        a.sort()

        b = (np.array(a[1:]) - np.array(a[:-1]))>50
        # print((b>1)[:400])
        ind = np.where(b == True)[0]
        # print(ind)
        # ts = [(a[0], a[ind[0]])]
        ts = []
        f = list(ind)
        f.extend([len(a)-1])
        g = [0]
        g.extend(ind+1)

        for i,k in zip(g, f):
            ts.append((a[i], a[k]))

        # print(ts)
        for n in range(1):
            startTime = ts[n][0]
            endTime = ts[n][1]
            # print(startTime - endTime)
            self.estimateLocation(startTime, endTime)

def main():
    rospy.init_node("Localization")
    # print(rospy.get_time())
    Localization()
    # Localization().test()
    rospy.spin()

if __name__=='__main__':
    main()