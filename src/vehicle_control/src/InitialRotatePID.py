from math import atan, sin, cos, radians, pi
class Controller():
    def __init__(self, orientation, start, goal,kPVel = 0.1, kIVel = 0.01, kDVel = 0.1, kPPos = 0.1, kIPos = 0.01, kDPos = 0.1, kPW = 0.1, kIW = 0.01, kDW = 0.1, timeDurationVel = 0.1, timeDurationPos=0.3, saturationWheel=100):
        self.startPossition = start
        self.goalPosition = goal
        self.goalThetha=atan(self.goalPosiition[1]-self.startPosition[1]/self.goalPosiition[0]-self.startPosition[0] )
        self.currentVelocity=[0,0]
        self.rightWheelVelocityGoal=0
        self.leftWheelVelocityGoal=0
        self.error = [0,0]  #Current Iteration error
        self.oldError = [0,0] # Previous error

        self.timeDurrationPos=0
        self.positionUpdated=False


        self.clampingLimit=0
        self.KpW = kPW
        self.KiW = kIW
        self.KdW = KdW
        
        self.KpPos = kPPos
        self.KiPos = kIPos
        self.KdPos = KdPos

        self.KpVel = kPVel
        self.KiVel = kIVel
        self.KdVel = KdVel
        self.prevErrorVel=[0,0]
        self.prevErrortheta=0
        self.thetaIntegError=0
        self.velIntegError=[0,0]
        self.turnRadius=0.5
        self.lengthOfRobot=0.4
        self.wheelRadius=0.1
        self.currentOrientation=orientation

    def getState():
        #do whatever to get current state
        #set current pos and velocity
        # velocity two array float right wheel, left wheel
        # orientation three array euler
        #current position x, y float
    def pidLoop():

        while !arived:
            self.getState()
            if self.currentPosition=self.goalPosition:
                return

            if self.rotationPhase==True:
                if(self.currentOrientation[0]==self.theta):
                    self.rotationPhase=False
                    continue
                error=self.theta-self.currentOrientation
                self.thetaIntegError= self.thetaIntegError + error * 0.1

                thetaDerivative = (error- self.prevErrortheta)/0.4

                output= self.KpW *error + self.KiW *self.thetaIntegError + thetaDerivative *self.KdW

                self.rightWheelVelocityGoal= (rotationalVelocity * self.lengthOfRobot)/ (2*self.wheelRadius)
                
                self.leftWheelVelocityGoal= ( -1 * rotationalVelocity * self.lengthOfRobot)/ (2*self.wheelRadius)
                self.prevErrortheta=error

            else:


                if self.positionUpdated:          
                    positionError= self.goalPosition - self.currentPosition
                    self.posIntegError= self.posIntegError + positionError * self.timeDurationPos

                    posDerivative = (positionError- self.prevErrorPos)/self.timeDurationPos

                    output= self.KpPos *positionError + self.KiPos *self.posIntegError +posDerivative *self.kDPos

                    self.prevErrorPos=positionError

                
                    v= (output[0] **2 + output[1]**2)**0.5

                    self.rightWheelVelocityGoal= (2 * v)/ (2*self.wheelRadius)
                
                    self.leftWheelVelocityGoal= self.rightWheelVelocityGoal
                    self.goalVelocity=[self.rightWheelVelocityGoal, self.leftWheelVelocityGoal]

                velError= self.goalVelocity- self.currentVelocity

                self.velIntegError= self.velIntegError + velIntegError * 0.1
                

                velDerivative = (velError- self.prevErrorVel)/0.1

                outputVel= self.KpVel *velError + self.KiVel *self.velIntegError + velDerivative *self.kDVel
                
                self.prevErrorVel=velError

                outPutRPM=numpy.array(outputVel)/(2*math.pi* self.wheelRadius)

                return outPutRPM







    




            



