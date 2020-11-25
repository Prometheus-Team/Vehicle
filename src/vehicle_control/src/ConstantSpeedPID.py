from math import atan, sin, cos, radians, pi
class Controller():
    def __init__(self, orientation, start, goal,kPVel = 0.1, kIVel = 0.01, kDVel = 0.1, kPPos = 0.1, kIPos = 0.01, kDPos = 0.1, timeDurationVel = 0.1, timeDurationPos=0.3, saturationWheel=100):
        self.prevPossition = start
        self.goalPosition = goal
        self.currentVelocity=[0,0]
        self.rightWheelVelocityGoal=0
        self.leftWheelVelocityGoal=0
        self.error = [0,0]  #Current Iteration error
        self.oldError = [0,0] # Previous error

        self.clampingLimit=0
        self.KpPos = kPPos
        self.KiPos = kIPos
        self.KdPos = KdPos

        self.KpVel = kPVel
        self.KiVel = kIVel
        self.KdVel = KdVel
        self.prevErrorVel=[0,0]
        self.prevErrorPos=[0,0]
        self.posIntegError=[0,0]
        self.velIntegError=[0,0]
        self.turnRadius=0.5
        self.lengthOfRobot=0.4
        self.wheelRadius=0.1

        self.currentOrientation=orientation

    def getState():
        #do whatever to get current state
        #set current pos and velocity

    def pidLoop():

        while !arived:
            self.getState()
            if self.currentPosition=self.goalPosition:
                return
            if(time%4==0):          
                positionError= self.goalPosition - self.currentPosition
                self.posIntegError= self.posIntegError + positionError * 0.4

                posDerivative = (positionError- self.prevErrorPos)/0.4

                output= self.KpPos *positionError + self.KiPos *self.posIntegError +posDerivative *self.kDPos

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
            
                v= (output[1] **2 + output[2]**2)**0.5

                self.rightWheelVelocityGoal= (2 * v + rotationalVelocity * self.lengthOfRobot)/ (2*self.wheelRadius)
            
                self.leftWheelVelocityGoal= (2 * v - rotationalVelocity * self.lengthOfRobot)/ (2*self.wheelRadius)
                self.goalVelocity=[self.rightWheelVelocityGoal, self.leftWheelVelocityGoal]

            velError= self.goalVelocity- self.currentVelocity

            self.velIntegError= self.velIntegError + velIntegError * 0.1
            

            velDerivative = (velError- self.prevErrorVel)/0.1

            outputVel= self.KpVel *velError + self.KiVel *self.velIntegError + velDerivative *self.kDVel
            
            self.prevErrorVel=velError

            outPutRPM=outputVel/(2*math.pi* self.wheelRadius)

            time+=1







    




            



