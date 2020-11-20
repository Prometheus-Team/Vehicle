

class DifferentialSteering:
    def __init__(self, radius, wheelGap):
        global readSpeedLeft, readSpeedRight

        self.radius = radius
        self.wheelGap = wheelGap
        self.speedToPWMConst = 4.5      # taking 40cm/s --> 180 PWM value
        self.vehicleSpeed = 40      # speed in cm/s

        self.speedLeft = 0
        self.speedRight = 0
        self.heading = 0

        self.path = {}

        

    def move(self, distance, direction, chosenPathToExecute):
        preDis = 0
        preOri = 0

        pidControl = PIDControl(0.1, 0.1, 0.1)
        pidControl.setTarget(0)

        while True:
            self.path['start'] = ('time','leftDistance','rightDistance','frontDistance','orientation')

            if self.path.get(chosenPathToExecute):
                self.path[chosenPathToExecute].append(('time','leftDistance','rightDistance','frontDistance','orientation'))
        
            else:
                self.path[chosenPathToExecute]=[('time','leftDistance','rightDistance','frontDistance','orientation')]

        self.path['end'] = ('time','leftDistance','rightDistance','frontDistance','orientation')

    def moveStraight(self, targetDistance, chosenPathToExecute):
        pointsInPath = []

        curHeading = self.heading

        pidControl = PIDControl(0.1, 0.1, 0.1)
        pidControl.setTarget(targetDistance)
        
        # while True:

            # pointsInPath.append(('time','leftDistance','rightDistance','frontDistance','orientation'))

            # out = pidControl.computeOutput()

        import time
        for i in range(5):
            time.sleep(1)
            self.pubSpeedLeft.publish(200)            
            self.pubSpeedRight.publish(200)

        self.pubSpeedLeft.publish(0)            
        self.pubSpeedRight.publish(0)            

    def turn(self, angle:str, tolerance=1):
        curAngle = self.getOrientation()

        pidControl = PIDControl(0.1, 0.1, 0.1)
        pidControl.setTarget(angle)

        while abs(pidControl.computeError(self.getOrientation())) > tolerance:
            err = abs(pidControl.computeError(self.getOrientation()))
            out = pidControl.computeOutput(err)


    def getCoveredDistance(self, path):
        init = path[0]
        end = path[-1]
        


    # direction can be +1 right, -1 left, 0 straight
    def computeSpeed(self, direction, innerSpeed):
        outerSpeed = ((self.radius+self.wheelGap)/self.radius)*innerSpeed
        if direction > 0:   # right direction
            return outerSpeed, innerSpeed

        elif direction < 0:     # left direction
            return innerSpeed, outerSpeed

        return innerSpeed, innerSpeed

    

    def computeError(self, preDis, dis, preOri, ori):
        return 0.5 * ((dis-preDis)**2 + (preOri-ori)**2)


    # converts speed given in cm/s to PWM voltage value
    def convertSpeedToPWM(self, speed):
        pwm = speed * self.speedToPWMConst
        if pwm > 255:
            return 255

        return pwm

    # change the PWM to speed ratio according to current movement of the vehicle
    def tuneSpeedToPWMConst(self, pwm, speed):
        self.speedToPWMConst = pwm/speed

    

class PIDControl:
    def __init__(self, P, I, D):
        self.P = P
        self.I = I
        self.D = D

        self.preErr = 0
        self.totErr = 0

        self.target = 0

    def setTarget(self, target):
        self.target = target

    def computeOutput(self, error):
        self.totErr += error
        output = -self.P * error - self.D * (error-self.preErr) - self.I * self.totErr
        self.preErr = error

        return output

    def computeError(self, curVal):
        return self.target - curVal
    


def main():
    v = DifferentialSteering(0.5, 0.3)
    v.moveStraight(5, None)

main()