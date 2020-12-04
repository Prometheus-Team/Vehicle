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
    def __init__(self, PIDParamVel, PIDParamPos,PIDParamW, timeDurationVel = 0.1, timeDurationPos=0.3, ):
        self.currentPosition = numpy.array([0.0,0.0])
        self.goalPosition = numpy.array([0.0,0.0])
        # self.goalThetha=atan(self.goalPosition[1]-self.currentPosition[1]/self.currentPosition[0]-self.currentPosition[0] )
        self.currentVelocity= numpy.array([0.0,0.0])
        self.rightWheelVelocityGoal=0.0
        self.leftWheelVelocityGoal=0.0
        self.error = numpy.array([0.0,0.0]) #Current Iteration error
        self.oldError = numpy.array([0.0,0.0]) # Previous error
        self.timeDurationVel=timeDurationVel
        self.timeDurationPos=0
        self.positionUpdated=False
        self.goalVelocity=[1.0,1.0]

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
        self.arrived=False
        self.time=0
        self.seqC = 0

        self.rotationPhase = False
        

        self.subSpeed = rospy.Subscriber('/pi/api/speed', Speed, self.speedCallback )
        self.subHeading = rospy.Subscriber('/pi/api/heading', Float32, self.headingCallback)
        self.pubCurLoc = rospy.Subscriber('/pi/localization/currentLoc', Location, self.curLocCallback)

        self.pubSpeed = rospy.Publisher('/pi/api/speedCmd', Speed, queue_size=10)

        self.gotoSrv = rospy.Service('/pi/steering/goto', Goto, self.gotoService)

        b = threading.Thread(target=self.pidLoop)
        b.start()

    def speedCallback(self, msg):
        self.currentVelocity = numpy.array([msg.right, msg.left])

    def headingCallback(self, msg):
        self.currentOrientation = msg.data

    def curLocCallback(self, msg):
        self.currentPosition = numpy.array([msg.x, msg.y])

    def sendSpeedCmd(self, speedRight, speedLeft):    
        s = Speed(left=speedLeft, right=speedRight, header=self.getHeader())
        self.pubSpeed.publish(s)

    def gotoService(self, msg):
        self.clear(numpy.array(msg.end))
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
        self.goalVelocity=[0.0,0.0]

    def rotationPhasePID(self):
        error=self.goalThetha-self.currentOrientation
        self.thetaIntegError= self.thetaIntegError + error * 0.1

        thetaDerivative = (error- self.prevErrortheta)/0.4

        outputW= self.PIDParamW.proportional *error + self.PIDParamW.integral *self.thetaIntegError + thetaDerivative *self.PIDParamW.derivative
        self.prevErrortheta=error

        self.rightWheelVelocityGoal= (outputW * self.lengthOfRobot)/ (2*self.wheelRadius)
        
        self.leftWheelVelocityGoal= ( -1 * outputW * self.lengthOfRobot)/ (2*self.wheelRadius)
        
    def positionPID(self):
        positionError= self.goalPosition - self.currentPosition
        self.posIntegError= self.posIntegError + positionError * self.timeDurationPos

        posDerivative = (positionError- self.prevErrorPos)/self.timeDurationPos

        output= self.PIDParamPos.proportional *positionError + self.PIDParamPos.integral *self.posIntegError +self.PIDParamPos.derivative *posDerivative


        self.prevErrorPos=positionError

    
        v= (output[0] **2 + output[1]**2)**0.5

        self.rightWheelVelocityGoal= (2 * v)/ (2*self.wheelRadius)
    
        self.leftWheelVelocityGoal= self.rightWheelVelocityGoal
        self.goalVelocity=[self.rightWheelVelocityGoal, self.leftWheelVelocityGoal]

    def velocityPID(self):
        velError= self.goalVelocity- self.currentVelocity

        self.velIntegError= self.velIntegError + velError * self.timeDurationVel
        
        velDerivative = (velError- self.prevErrorVel)/self.timeDurationVel

        outputVel= self.PIDParamVel.proportional *velError + self.PIDParamVel.integral *self.velIntegError + self.PIDParamVel.derivative *velDerivative
        outputVel += self.currentVelocity
        self.prevErrorVel=velError

        #outPutRPM=numpy.array(outputVel)/(2*math.pi* self.wheelRadius)
        self.sendSpeedCmd(outputVel[0],outputVel[1])

    def pidLoop(self):

        while not self.arrived:
            # if self.currentPosition==self.goalPosition:
            #     outputVel=[0.0,0.0]
            #     self.arrived=True
            #     self.sendSpeedCmd(outputVel[0],outputVel[1])

            #     return True

            if self.rotationPhase==True:
                if self.currentOrientation==self.goalThetha:
                    self.rotationPhase=False
                    continue
                
                self.rotationPhasePID()
                

            else:
                #if self.time%3==0:          
                    #self.positionPID()
                self.velocityPID()

                self.time+=1
                sleep(0.1)

        return False


def main():
    rospy.init_node("InitialRotatePID")
    pidParamVel=PIDParam(1.0,0,0)
    pidParamPos=PIDParam(0.1,0.01,0.1)
    pidParamW=PIDParam(0.1,0.01,0.1)
    a = Controller(pidParamVel,pidParamPos,pidParamW)
    a.goalVelocity = [1.0,1.0]
    rospy.spin()

if __name__=='__main__':
    main()