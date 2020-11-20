#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32, Int16, Header, Time, Empty, Int32
from sensor_msgs.msg import Range
from geometry_msgs.msg import Point
from vehicle_lib.msg import Distance, Speed, Steps
from quaternion import Quaternion
import pyrr, urllib, json, time, math, threading


class VehicleAPI:
    URL = "http://192.168.1.103:8080/sensors.json"
    def __init__(self):
        self.seqC = 0       # sequence number for the streamed values
        self.defaultScannerStep = 2 # 2 degrees rotated on every step
        self.scannerInitAngle = 90      # servo motor initial angle
        self.speedToPWMRatio = 255.0/40.0        
        self.minDistanceToObstacle = 50 # in centimeters
        self.maxTimeForValidSpeed = 0.7 # in seconds
        self.lastSpeedCmd = (0, 0)

        self.speedLeft = self.speedRight = self.stepLeft = self.stepRight = self.heading = 0
        self.timeLeft = self.timeRight = 0
        self.totLeftStep = self.totRightStep = 0
        self.mapper = None

        head = threading.Thread(target=self.readOrientation)
        # self.rate = rospy.Rate(1)

        self.pubSpeedArd = rospy.Publisher('/arduino/speedCmd', Point, queue_size=10)
        # self.pubSpeedRight = rospy.Publisher('/arduino/speed/rightCmd', Float32, queue_size=10)
        self.pubScan = rospy.Publisher('/arduino/scanCmd', Int16, queue_size=10)

        self.subSpeedLeft = rospy.Subscriber('/arduino/speed/left', Float32, self.readSpeedLeft)
        self.subSpeedRight = rospy.Subscriber('/arduino/speed/right', Float32, self.readSpeedRight)
        self.subStepLeft = rospy.Subscriber('/arduino/step/left', Int32, self.readStepLeft)
        self.subStepRight = rospy.Subscriber('/arduino/step/right', Int32, self.readStepRight)
        self.subRange = rospy.Subscriber('/arduino/range', Range, self.readRange)
        self.subObstacleDistance = rospy.Subscriber('/arduino/obstacleDistance', Float32, self.checkObstacle)
        

        # This topics will be used by the higher level API
        # ==================================================
        self.pubRange = rospy.Publisher('/pi/api/range', Distance, queue_size=10)
        self.pubHeading = rospy.Publisher('/pi/api/heading', Float32, queue_size=10)
        self.pubSpeed = rospy.Publisher('/pi/api/speed', Speed, queue_size=10)
        self.pubStep = rospy.Publisher('/pi/api/step', Steps, queue_size=10)
        self.pubObstacle = rospy.Publisher('/pi/api/obstacleDistance', Float32, queue_size=10)
        self.pubFrontDis = rospy.Publisher('/pi/api/frontDistance', Distance, queue_size=10)

        self.subRange = rospy.Subscriber('/pi/api/scanCmd', Empty, self.scanAreaCmd)
        self.subSpeed = rospy.Subscriber('/pi/api/speedCmd', Speed, self.speedCmd)
        # ==================================================

        head.start()

    def initConfig(self):
        # Make the range sensor face to the front (90 deg)
        # Compute speedToPWMRatio
        # Turn the vehicle to face the North ????????
        pass

    # Speed left and right should be in PWM
    def speedCmd(self, speed):
        if int(time.time()) - speed.header.stamp.secs <= 2:
            self.lastSpeedCmd = (speed.left, speed.right)

            if speed.left == 0 or speed.right == 0:
                self.pubSpeedArd.publish(Point(x=0, y=0))
            else:
                self.pubSpeedArd.publish(Point(x=255*speed.left/abs(speed.left), y=255*speed.right/abs(speed.right)))
        
    # inititate the servo rotation to scan the area for 180deg
    def scanAreaCmd(self, msg):
        self.pubScan.publish(self.defaultScannerStep)

    def readSpeedLeft(self, msg):
        # if self.speedLeft > 0 and abs(self.speedLeft - msg.data) > self.speedLeft*0.2:
        #     return

        if time.time() - self.timeLeft > self.maxTimeForValidSpeed:
            self.speedLeft = msg.data/60.0

        else:
            self.speedLeft = 0.5 * self.speedLeft + 0.5 * (msg.data/60)

        self.timeLeft = time.time()

        if time.time() - self.timeRight < self.maxTimeForValidSpeed:
            self.pubSpeed.publish(Speed(self.speedLeft, 0, self.getHeader(), self.lastSpeedCmd[0], self.lastSpeedCmd[1]))
    
        else:
            self.pubSpeed.publish(Speed(self.speedLeft, self.speedRight, self.getHeader(), self.lastSpeedCmd[0], self.lastSpeedCmd[1]))

    def readSpeedRight(self, msg):

        # if self.speedRight > 0.2 and abs(self.speedRight - msg.data) > self.speedRight*0.2:
        #     return

        if time.time() - self.timeRight > self.maxTimeForValidSpeed:
            self.speedRight = msg.data/60.0
        
        else:
            self.speedRight = 0.5 * self.speedRight + 0.5 * (msg.data/60.0)

        self.timeRight = time.time()

        if time.time() - self.timeLeft < self.maxTimeForValidSpeed:
            self.pubSpeed.publish(Speed(self.speedLeft, self.speedRight, self.getHeader(), self.lastSpeedCmd[0], self.lastSpeedCmd[1]))
        
        else:
            self.pubSpeed.publish(Speed(0, self.speedRight, self.getHeader(), self.lastSpeedCmd[0], self.lastSpeedCmd[1]))

    def readStepLeft(self, msg):
        self.totLeftStep += 1
        self.pubStep.publish(Steps(self.totLeftStep, self.totRightStep, self.getHeader()))

    def readStepRight(self, msg):
        self.totRightStep += 1
        self.pubStep.publish(Steps(self.totLeftStep, self.totRightStep, self.getHeader()))

    def readRange(self, msg):
        self.pubRange.publish(Distance(msg.field_of_view, msg.range, msg.radiation_type==1, self.getHeader()))      

    def readOrientation(self):
        import tf
        while True:
            response = urllib.urlopen(self.URL)
            data=json.loads(response.read())
            i=data['rot_vector']['data'][-1]
            currentQuaternion=pyrr.quaternion.create(i[1][0],i[1][1],i[1][2],i[1][3])
            # self.heading = tf.transformations.euler_from_quaternion([i[1][0],i[1][1],i[1][2],i[1][3]])[2]*180/math.pi
            self.heading = Quaternion(currentQuaternion).to_euler123()[0]*180/math.pi
            print(self.heading)
            self.pubHeading.publish(self.heading)

    def computeSpeedToPWMRatio(self, pwm, speed):
        return pwm/speed

    def checkObstacle(self, distance):
        self.pubFrontDis.publish(Distance(90, distance.data, False, self.getHeader()))
        if distance.data < self.minDistanceToObstacle:
            print("Collision imminent")
            self.pubObstacle.publish(distance.data)

    def getHeader(self, seq=0):
        self.seqC += 1
        v = Header(self.seqC, None, '1')
        v.stamp = rospy.Time.now()
        return v

def main():
    rospy.init_node("VehicleAPI")
    VehicleAPI()
    rospy.spin()

if __name__=='__main__':
    main()
