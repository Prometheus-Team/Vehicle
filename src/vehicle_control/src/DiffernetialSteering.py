#!/usr/bin/env python

import rospy
from vehicle_lib.msg import Location, Speed

from time import sleep

from math import atan, sin, cos, radians, pi
from PIDParam import PIDParam 
class DifferentialSteering():
    def __init__(self, orientation, start, goal,PIDParamVel, PIDParamPos, timeDurationVel = 0.1, timeDurationPos=0.3, saturationWheel=100, turnRadius=0.5, lenthOfRobot,wheelRadius=0.3):
        self.prevPossition = start #Two Dimensional Array Representing position in X and Y Coordinate system
        self.goalPosition = goal
        self.currentVelocity=[0,0]
        self.currentPosition=[0,0]
        self.currentOrientation=orientation
        self.rightWheelVelocityGoal=0
        self.leftWheelVelocityGoal=0

        self.clampingLimit=saturationWheel
        self.PIDParamPos=PIDParamPos
        self.PIDParamVel=PIDParamVel

        self.timeDurationPos=timeDurationPos
        self.timeDurationVel=timeDurationVel

        self.prevErrorVel=[0,0]
        self.prevErrorPos=[0,0]

        self.posIntegError=[0,0]
        self.velIntegError=[0,0]

        self.turnRadius=turnRadius
        self.lengthOfRobot=lengthOfRobot
        self.wheelRadius=wheelRadius

        self.seqC = 0
        self.arrived=False
        self.time=0
        self.subSpeed = rospy.Subscriber('/pi/api/speed', Speed, self.speedCallback )
        self.subHeading = rospy.Subscriber('/pi/api/heading', Float32, self.headingCallback)
        self.pubCurLoc = rospy.Subscriber('/pi/localization/currentLoc', Location, self.curLocCallback)

        self.pubSpeed = rospy.Publisher('/pi/api/speedCmd', Speed, queue_size=10)
        self.pidLoop()

    def speedCallback(self, msg):
        self.currentVelocity = [msg.right, msg.left]

    def headingCallback(self, msg):
        self.currentOrientation = msg.data

    def curLocCallback(self, msg):
        self.currentPosition = [msg.x, msg.y]

    def sendSpeedCmd(self, speedRight, speedLeft):    
        s = Speed(left=speedLeft, right=speedRight, header=self.getHeader())
        self.pubSpeed.publish(s)

    def getHeader(self):
        self.seqC += 1
        v = Header(self.seqC, None, '1')
        v.stamp = rospy.Time.now()
        return v

    def getState(self):
        #do whatever to get current state
        #set current pos and velocity
        pass

    def pidLoop(self):
        while not self.arrived:
            if self.currentPosition==self.goalPosition:
                outputVel=[0.0,0.0]
                self.arrived=True
                
            self.sendSpeedCmd(outputVel[0],outputVel[1])
            if self.time%4==0:          
                positionError= self.goalPosition - self.currentPosition
                self.posIntegError= self.posIntegError + positionError * self.timeDurationPos

                posDerivative = (positionError- self.prevErrorPos)/self.timeDurationPos

                output= self.PIDParamPos.proportional *positionError + self.PIDParamPos.integral *self.posIntegError +self.PIDParamPos.derivative *self.kDPos

                self.prevErrorPos=positionError

                theta=atan(output[1]/output[0])

                difference= theta-self.currentOrientation[2] 


                xComp1=output[0]*math.cos(difference)
                yComp1=output[0]*math.sin(difference)

                yComp2=output[1]*math.cos(difference)
                xComp2=output[1]*math.sin(difference)

                xCompTotal=xComp1+xComp2

                yCompTotal=yComp1+yComp2

                rotationalVelocity=xCompTotal/self.turnRadius
            
                v= (output[0] **2 + output[1]**2)**0.5

                self.rightWheelVelocityGoal= (2 * v + rotationalVelocity * self.lengthOfRobot)/ (2*self.wheelRadius)
            
                self.leftWheelVelocityGoal= (2 * v - rotationalVelocity * self.lengthOfRobot)/ (2*self.wheelRadius)
                self.goalVelocity=[self.rightWheelVelocityGoal, self.leftWheelVelocityGoal]

            velError= self.goalVelocity- self.currentVelocity

            self.velIntegError= self.velIntegError + velIntegError * self.timeDurationVel
            

            velDerivative = (velError- self.prevErrorVel)/self.timeDurationVel

            outputVel= self.PIDParamVel.proportional *velError + self.PIDParamVel.integral *self.velIntegError + self.PIDParamVel.derivative *self.kDVel
            
            self.prevErrorVel=velError


            self.time+=1
            #outPutRPM=outputVel/(2*math.pi* self.wheelRadius)
           self.sendSpeedCmd(outputVel[0],outputVel[1])



def main():
    rospy.init_node("DifferentialSteering")
    DifferentialSteering()
    rospy.spin()

if __name__=='__main__':
    main()






    




            


