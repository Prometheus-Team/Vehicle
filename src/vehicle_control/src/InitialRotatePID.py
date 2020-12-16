#!/usr/bin/env python

import rospy
from vehicle_lib.msg import Location, Speed
from vehicle_lib.srv import Goto
from std_msgs.msg import Float32, Header

from time import sleep
from math import atan, sin, cos, radians, pi
from PIDParam import PIDParam 
import numpy

import threading

class Controller:
    def __init__(self, PIDParamVel, PIDParamPos,PIDParamW, timeDurationVel = 0.1, timeDurationPos=0.3 ):
        self.currentPosition = numpy.array([0.0,0.0])
        self.goalPosition = numpy.array([0.0,0.0])
        self.goalThetha=atan(self.goalPosition[1]-self.currentPosition[1]/self.currentPosition[0]-self.currentPosition[0] )
        self.currentVelocity= numpy.array([0.0,0.0])
        self.rightWheelVelocityGoal=0.0
        self.leftWheelVelocityGoal=0.0
        self.error = numpy.array([0.0,0.0]) #Current Iteration error
        self.oldError = numpy.array([0.0,0.0]) # Previous error
        self.timeDurationVel=timeDurationVel
        self.timeDurationPos=timeDurationPos
        self.positionUpdated=False


        self.clampingLimit=0
        self.PIDParamW=PIDParamW
        
        self.PIDParamPos=PIDParamPos
        self.PIDParamVel=PIDParamVel
        self.prevErrorPos=numpy.array([0.0,0.0])
        self.prevErrorVel= numpy.array([0.0,0.0])
        self.prevErrortheta=0.0
        self.thetaIntegError=0.0
        self.velIntegError= numpy.array([0.0,0.0])
        self.posIntegError=numpy.array([0.0,0.0])
        self.turnRadius=0.5
        self.lengthOfRobot=0.4
        self.wheelRadius=0.1
        
        self.currentOrientation=0
        self.relativeOrientation = 0
        self.initialOrientation = None

        self.arrived=False
        self.time=0
        self.seqC = 0

        self.rotationPhase = True
        

        self.subSpeed = rospy.Subscriber('/pi/api/speed', Speed, self.speedCallback )
        self.subHeading = rospy.Subscriber('/pi/api/heading', Float32, self.headingCallback)
        self.pubCurLoc = rospy.Subscriber('/pi/localization/currentLoc', Location, self.curLocCallback)

        self.pubSpeed = rospy.Publisher('/pi/api/speedCmd', Speed, queue_size=10)

        self.gotoSrv = rospy.Service('/pi/steering/goto', Goto, self.gotoService)

    def speedCallback(self, msg):
        self.currentVelocity = numpy.array([msg.right, msg.left])

    def headingCallback(self, msg):
        self.currentOrientation = msg.data
        if self.initialOrientation:
            self.relativeOrientation = self.initialOrientation - self.currentOrientation

    def curLocCallback(self, msg):
        self.currentPosition = numpy.array([msg.x, msg.y])

    def sendSpeedCmd(self, speedRight, speedLeft):    
        s = Speed(left=speedLeft, right=speedRight, header=self.getHeader())
        self.pubSpeed.publish(s)

    def gotoService(self, msg):
        if self.initialOrientation is None:
            self.initialOrientation = self.currentOrientation

        self.clear(numpy.array([msg.end.x, msg.end.y]))
        return self.pidLoop()

    def getHeader(self):
        self.seqC += 1
        v = Header(self.seqC, None, '1')
        v.stamp = rospy.Time.now()
        return v

    def getState(self):
        #do whatever to get current state
        #set current pos and velocity
        # velocity two array float right wheel, left wheel
        # orientation three array euler
        #current position x, y float
        pass

    def clear(self,goalPosition):
        self.goalPosition=goalPosition
        print(self.goalPosition)
        print(self.currentPosition)
        if self.goalPosition[0]-self.currentPosition[0]==0:
            self.goalThetha=0
        else:
            self.goalThetha=atan(self.goalPosition[1]-self.currentPosition[1]/self.currentPosition[0]-self.currentPosition[0] )
        self.prevErrorPos=numpy.array([0.0,0.0])
        self.prevErrorVel= numpy.array([0.0,0.0])
        self.prevErrortheta=0.0
        self.thetaIntegError=0.0
        self.velIntegError= numpy.array([0.0,0.0])
        self.prevErrorPos=numpy.array([0.0,0.0])
        self.posIntegError=numpy.array([0.0,0.0])
        self.arrived = False
        self.rotationPhase=True

    def rotationPhasePID(self):
        error=self.goalThetha-self.relativeOrientation
        self.thetaIntegError= self.thetaIntegError + error * 0.1
        print(self.relativeOrientation)
        thetaDerivative = (error- self.prevErrortheta)/0.4

        outputW= self.PIDParamW.proportional *error + self.PIDParamW.integral *self.thetaIntegError + thetaDerivative *self.PIDParamW.derivative
        self.prevErrortheta=error

        self.rightWheelVelocityGoal= (outputW * self.lengthOfRobot)/ (2*self.wheelRadius)
        
        self.leftWheelVelocityGoal= ( -1 * outputW * self.lengthOfRobot)/ (2*self.wheelRadius)
        
        self.sendSpeedCmd(self.rightWheelVelocityGoal, self.leftWheelVelocityGoal)

    def positionPID(self):
        self.goalPosition=self.goalPosition.astype(numpy.float32)
        self.currentPosition=self.currentPosition.astype(numpy.float32)
        positionError= numpy.subtract(self.goalPosition ,self.currentPosition)
        self.posIntegError= self.posIntegError + positionError * self.timeDurationPos
        #print(positionError)
        self.prevErrorPos=self.prevErrorPos.astype(numpy.float32)
        posDerivative = numpy.subtract(positionError, self.prevErrorPos)*(1/self.timeDurationPos)
        #print(posDerivative)

        output= self.PIDParamPos.proportional *positionError + self.PIDParamPos.integral *self.posIntegError +self.PIDParamPos.derivative *posDerivative

        self.prevErrorPos=positionError

    
        v= (output[0] **2 + output[1]**2)**0.5
        if output[0]<0:
            self.rightWheelVelocityGoal= ((2 * v)/ (2*self.wheelRadius)) *-1.0
        
            self.leftWheelVelocityGoal= self.rightWheelVelocityGoal
            self.goalVelocity=[self.rightWheelVelocityGoal, self.leftWheelVelocityGoal]
        else: 
            self.rightWheelVelocityGoal= ((2 * v)/ (2*self.wheelRadius))
        
            self.leftWheelVelocityGoal= self.rightWheelVelocityGoal
            self.goalVelocity=[self.rightWheelVelocityGoal, self.leftWheelVelocityGoal]

        return self.goalVelocity

    def velocityPID(self):
        velError= self.goalVelocity- self.currentVelocity

        self.velIntegError= self.velIntegError + velError * self.timeDurationVel
        
        velDerivative = (velError- self.prevErrorVel)/self.timeDurationVel

        outputVel= self.PIDParamVel.proportional *velError + self.PIDParamVel.integral *self.velIntegError + self.PIDParamVel.derivative *velDerivative
    
        self.prevErrorVel=velError

        #outPutRPM=numpy.array(outputVel)/(2*math.pi* self.wheelRadius)
        self.time+=1
        self.sendSpeedCmd(outputVel[0],outputVel[1])

    def pidLoop(self):

        while not self.arrived:
            if self.currentPosition[0]==self.goalPosition[0] and self.currentPosition[1]==self.goalPosition[1]:
                outputVel=[0.0,0.0]
                self.arrived=True
                self.sendSpeedCmd(outputVel[0],outputVel[1])
                print(self.currentPosition)
                return True

            if self.rotationPhase==True:
                print("Here",self.relativeOrientation, self.goalThetha)
                # return
                if self.currentOrientation==self.goalThetha:
                    self.rotationPhase=False
                    continue
                
                self.rotationPhasePID()
                

            else:
                if self.time%4==0:          
                    self.positionPID()
                self.velocityPID()
            sleep(1/3)

        return False


def main():
    rospy.init_node("InitialRotatePID")
    pidParamVel=PIDParam(1,0,0)
    pidParamPos=PIDParam(0.1,0.01,0.01)
    pidParamW=PIDParam(0.001,0.001,0.0001)
    Controller(pidParamVel,pidParamPos,pidParamW)
    rospy.spin()

if __name__=='__main__':
    main()