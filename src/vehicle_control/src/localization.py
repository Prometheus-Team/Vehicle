#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32, Int16
from sensor_msgs.msg import Range
from vehicle_lib.msg import Distance, Speed, Location, Steps
from quaternion import Quaternion
import pyrr, urllib, json, time, math, threading

import numpy as np
import enum, pickle, math

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
        diff.steps = (self.steps[0]-preData.steps[0], self.steps[1]-preData.steps[1])

        # acc = self.computeAccelration(preData)
        # diff.leftAcceleration = acc[0]
        # diff.rightAcceleration = acc[1]

        diff.direction = self.computeDirectionRotate(preData)

        return diff

    def computeAccelration(self, preData):
        rightAcc = (self.speedRight[0] - preData.speedRight[0])/((self.timestamp - preData.timestamp)/1000.0)
        leftAcc = (self.speedLeft[0] - preData.speedLeft[0])/((self.timestamp - preData.timestamp)/1000.0)
        
        return leftAcc, rightAcc 

    # Computes direction when vehicle is allowed to turn direction smoothly around a curve
    def computeDirectionCurved(self, preData):
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

    # Computes direction when vehicle changes direction by rotating in its own axis
    def computeDirectionRotate(self, preData):
        if self.speedLeft[1] == 0 and self.speedRight[1] == 0:
            return Direction.Stationary

        elif self.speedLeft[1] > 0 and self.speedRight[1] > 0:
            return Direction.Forward

        elif self.speedLeft[1] > 0 and self.speedRight[1] < 0:
            return Direction.Right

        elif self.speedLeft[1] < 0  and self.speedRight[1] > 0:
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

    def updateLoc(self, x, y, heading, timestamp):
        self.x += x
        self.y += y
        self.heading = heading
        self.timestamp = timestamp

    def getLoc(self):
        return Location(self.x, self.y, self.heading, None)


class Prediction:
    def __init__(self, speedDis, stepDis, rangeDis, heading, direction):
        self.speedEff = 0.7
        self.stepEff = 0.4
        self.rangeEff = 0.5

        self.speedDis = speedDis
        self.stepDis = stepDis
        self.rangeDis = rangeDis
        self.imuDis = None
        self.heading = heading
        self.direction = direction      # Forward, backward, left, right

    def combine(self):
        avg = (self.speedDis + self.stepDis + self.rangeDis) / 3

        return avg

    def getCoord(self, distance):
        # xSpeed = math.sin(math.radians(self.heading)) * distance
        # ySpeed = math.cos(math.radians(self.heading)) * distance

        x = math.sin(math.radians(self.heading)) * distance
        y = math.cos(math.radians(self.heading)) * distance
        
        # if self.rangeDis < 5:
        #     xRange = math.sin(math.radians(self.heading)) * distance
        #     yRange = math.cos(math.radians(self.heading)) * distance

        #     return ((xSpeed, ySpeed), (xStep, yStep), (xRange, yRange))

        # return ((xSpeed, ySpeed), (xStep, yStep), None)
        return x, y


class Localization:
    def __init__(self):
        self.dataRecordFreq = 50        # number of times we record data per second
        self.maxValidTime = 50
        self.wheelRadius = 0.033
        self.wheelGap = 0.14    # in meters
        self.revToStepRatio = 20.0    # 1 rev = 20 steps
        self.distanceDiffTolerance = 0.05   # in meters
        self.headingDiffTolerance = 2   # in angles

        self.lastKnownLocation = LastLocation(0,0,0,self.getCurTimeInMilliSecs())
        self.inMotion = 0
        self.locationComputed = False

        self.heading = 0
        self.range = 0
        self.speedLeft = (0, 0)
        self.speedRight = (0, 0)
        self.steps = (0, 0)
        self.imu = None

        self.travelTrace = {}
        self.travelHistory = {}

        self.pubCurLoc = rospy.Publisher('/pi/localization/currentLoc', Location, queue_size=10)

        self.subFrontDistance = rospy.Subscriber('/pi/api/frontDistance', Distance, self.frontDistanceCallback)
        self.subHeading = rospy.Subscriber('/pi/api/heading', Float32, self.headingCallback)
        self.subSpeed = rospy.Subscriber('/pi/api/speed', Speed, self.speedCallback )
        self.subStep = rospy.Subscriber('/pi/api/step', Steps, self.stepsCallback )

        # self.recordData()
        dataRecording = threading.Thread(target=self.recordData)
        dataRecording.daemon = True
        dataRecording.start()
        dataRecording.join()

    def recordData(self):
        locationComputedTime = time.time()
        while True:
            isMoving = self.getCurTimeInMilliSecs() - self.inMotion < 200
            isTimeToComputeLocation = self.getCurTimeInMilliSecs() - locationComputedTime > 500
            # Record motion state of vehicle if its in motion
            if isMoving:
                self.locationComputed = False
                # print("REcord")

                t = self.getCurTimeInMilliSecs()
                tra = TravelData(t, self.getHeading(), self.getRange(), self.getSpeedLeft(), self.getSpeedRight(), self.getSteps(), self.getIMU())
                self.travelTrace[t] = tra

                time.sleep(1/self.dataRecordFreq)

            # Compute current location once the vehicle becomes stationary
            if len(self.travelTrace) > 0 and self.locationComputed is False and isMoving and isTimeToComputeLocation:
                self.locationComputed = True
                locationComputedTime = self.getCurTimeInMilliSecs()
                # print("Estimate")
                self.estimateLocation(self.lastKnownLocation.timestamp, self.inMotion)

                # Store the current uninterrupted travel history in the archive
                self.archiveTravelTrace()

            # Quit if q is pressed
            if 0xFF == ord('q'):
                return

    def headingCallback(self, msg):
        self.heading = msg.data
        if self.lastKnownLocation.timestamp == 0:
            self.lastKnownLocation.heading = self.heading
            self.lastKnownLocation.timestamp = self.getCurTimeInMilliSecs()

    def frontDistanceCallback(self, msg):
        self.range = msg.distance

    def speedCallback(self, msg):
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

    def archiveTravelTrace(self):
        a = self.travelTrace.keys()
        a.sort()

        self.travelHistory[(a[0], a[-1])] = self.travelTrace
        self.travelTrace = {}

    def estimateLocation(self, startTime, endTime):
        import sys
        preds = []
        times = self.getTravelData(startTime, endTime)
        v = self.manipulateData(times)
        ts = v.keys()
        ts.sort()

        for tStart, tEnd in zip(ts[:-1], ts[1:]):
            if tEnd - tStart <= self.maxValidTime:
                dataStart = self.travelTrace.get(tStart)
                dataEnd = self.travelTrace.get(tEnd)

                diff = dataEnd.getDiff(dataStart)

                # distance from speedometer
                # leftDisSpeed = self.computeDistanceFromAccel(dataStart.speedLeft[0], diff.leftAcceleration, diff.timestamp/1000.0)
                # rightDisSpeed = self.computeDistanceFromAccel(dataStart.speedRight[0], diff.rightAcceleration, diff.timestamp/1000.0)
                # speedDis = self.combineDistance(leftDisSpeed, rightDisSpeed, diff.heading)

                # distance from steps
                leftDisStep = self.computeDistanceFromSteps(diff.steps[0])
                rightDisStep = self.computeDistanceFromSteps(diff.steps[1])
                stepDis = self.combineDistance(leftDisStep, rightDisStep, diff.heading)

                pred = Prediction(0, stepDis, diff.range, diff.heading, diff.direction)
                preds.append(pred)

        # print(len(preds), len(times))
        print(startTime, endTime)
        if len(preds) > 0:
            x1, y1 = self.combineLoc(preds)

            stS = self.travelTrace.get(times[0]).steps
            stL = self.travelTrace.get(times[-1]).steps
            headS = self.travelTrace.get(times[0]).heading
            headL = self.travelTrace.get(times[-1]).heading
            angle = headL - headS

            cir = 2*math.pi*self.wheelRadius
            d = ((((stL[0] - stS[0])/20.0)*cir) + (((stL[1] - stS[1])/20.0)*cir))/2.0
            xStep = math.sin(math.radians(headL)) * d
            yStep = math.cos(math.radians(headL)) * d

            # # d = (((stL[1] - stS[1]-20)/20.0)*0.207)
            # print(stL[0]- stS[0], stL[1]- stS[1])
            # print(x,y,x1,y1)
            
            # print("Distance Exact: ", d, ((stL[0] - stS[0]) + (stL[1] - stS[1]))/200.0, xStep, yStep)
            print("Distance: ", d," Angle:", angle,"X:", xStep,"Y:", yStep)
            
            self.updateCurrentLoc(xStep, yStep, headL)

            # if abs(y1) > 0.05:
            #     self.updateCurrentLoc(x1, (abs(y1)/y1)*d, headL)
            # else:
            #     self.updateCurrentLoc(x1, 0, headL)

            print("Absolute Location: ", self.lastKnownLocation.x, self.lastKnownLocation.y, self.lastKnownLocation.heading)
            # sys.exit()

    def combineLoc(self, preds):
        xSp=xSt = 0
        ySp=ySt = 0

        for pred in preds:
            # coordSp = pred.getCoord(pred.speedDis)
            x, y = pred.getCoord(pred.stepDis)

            if pred.direction == Direction.Backward:
                # xSp += -coordSp[0][0]
                # ySp += -coordSp[0][1]
                xSt += -x
                ySt += -y
                
            elif pred.direction == Direction.Forward:
                # xSp += coordSp[0][0]
                # ySp += coordSp[0][1]
                xSt += x
                ySt += y

        # return xSp, ySp, xSt, ySt
        return xSt, ySt

    def updateCurrentLoc(self, x, y, heading):
        self.lastKnownLocation.updateLoc(x, y, heading, self.inMotion)

        # Publish updated location to current location topic
        self.pubCurLoc.publish(self.lastKnownLocation.getLoc())

    def computeDistanceFromAccel(self, vi, acc, t):
        return vi * t + (0.5 * acc * t**2)

    def computeDistanceFromSteps(self, stepDiff):
        return 2 * math.pi * self.wheelRadius * (stepDiff/self.revToStepRatio)

    def combineDistance(self, left, right, angle):
        if angle < self.headingDiffTolerance or abs(left-right) <= self.distanceDiffTolerance:
            return (left+right)/2

        # elif left < right and angle < 0:
        #     rIn = left/abs(angle)
        #     rOut = (right/abs(angle)) - self.wheelGap

        # elif left > right and angle > 0:
        #     rIn = right/angle
        #     rOut = (left/angle) - self.wheelGap

        #r = rIn + rOut
        r = (left/abs(angle)) + (right/abs(angle)) - self.wheelGap

        # return (2 * math.sin(math.radians(angle)/2) * (rIn + rOut))/2
        return math.sin(math.radians(angle)/2) * r

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
    # print("Spinning ================================================================")
    rospy.spin()

if __name__=='__main__':
    main()