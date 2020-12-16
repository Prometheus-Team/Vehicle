#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32, Int16, Header, Time, Empty, Int32
from sensor_msgs.msg import Range
from geometry_msgs.msg import Point
from vehicle_lib.msg import Distance, Speed, Steps
from vehicle_lib.srv import ScanArea
from quaternion import Quaternion
import pyrr, urllib, json, time, math, threading


class VehicleAPI:
    # URL = "http://192.168.0.101:8080/sensors.json"
    def __init__(self, phoneURL, speed, pwm):
        self.URL = phoneURL
        
        self.seqC = 0       # sequence number for the streamed values
        self.defaultScannerStep = 2 # 2 degrees rotated on every step
        self.scannerInitAngle = 90      # servo motor initial angle
        self.speedToPWMRatio = self.computeSpeedToPWMRatio(pwm,speed)        
        self.minDistanceToObstacle = 50 # in centimeters
        self.maxTimeForValidSpeed = 0.7 # in seconds
        self.lastSpeedCmd = (0, 0)

        self.speedLeft = self.speedRight = self.stepLeft = self.stepRight = self.heading = 0
        self.timeLeft = self.timeRight = 0
        self.totLeftStep = self.totRightStep = 0
        self.curFactor = 0.3
        self.preFactor = 0.7
        self.inMotion = time.time()

        self.lastStepLeft = self.lastStepRight = (0,0)
        self.lastSpeedLeft = self.lastSpeedRight = 0

        head = threading.Thread(target=self.readOrientation)
        head.setDaemon(True)
        # self.rate = rospy.Rate(1)

        self.pubSpeedArd = rospy.Publisher('/arduino/speedCmd', Point, queue_size=10)
        self.pubScan = rospy.Publisher('/arduino/scanCmd', Int16, queue_size=10)

        self.subSpeed = rospy.Subscriber('/arduino/speed', Point, self.readSpeed)
        self.subStep = rospy.Subscriber('/arduino/step', Point, self.readStep)
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

        self.scanAreaSrv = rospy.Service('/pi/api/scanCmd', ScanArea, self.scanAreaCmd)
        self.subSpeed = rospy.Subscriber('/pi/api/speedCmd', Speed, self.speedCmd)
        # ==================================================
 
        head.start()
        # self.readOrientation()

    def initConfig(self):
        # Make the range sensor face to the front (90 deg)
        # Compute speedToPWMRatio
        # Turn the vehicle to face the North ????????
        pass

    def convertSpeedToPWM(self, speed):
        pwm = speed * self.speedToPWMRatio
        if abs(pwm) > 255:
            return 255 * pwm/abs(pwm)

        elif abs(pwm) < 60 and pwm!=0:
            return 60 * pwm/abs(pwm)

        return pwm

    # Speed left and right should be in PWM
    def speedCmd(self, speed):
        if int(time.time()) - speed.header.stamp.secs <= 2:
            if speed.left != 0 and speed.right != 0:
                self.lastSpeedCmd = (speed.left, speed.right)

            if speed.left == 0 or speed.right == 0:
                self.pubSpeedArd.publish(Point(x=0, y=0))
            else:
                self.pubSpeedArd.publish(Point(x=self.convertSpeedToPWM(speed.left), y=self.convertSpeedToPWM(speed.right)))
        
    # inititate the servo rotation to scan the area for 180deg
    def scanAreaCmd(self, msg):
        try:
            self.pubScan.publish(self.defaultScannerStep)
            return True

        except:
            return False

    def readSpeed(self, msg):
        left = msg.x
        right = msg.y
        
        if self.lastSpeedCmd[0] < 0:
            left *= -1

        if self.lastSpeedCmd[1] < 0:
            right *= -1 

        if time.time() - self.inMotion > 0.2:
            self.speedLeft = left
            self.speedRight = right

        else:
            self.speedLeft = self.preFactor * self.speedLeft + self.curFactor * left
            self.speedRight = self.preFactor * self.speedRight + self.curFactor * right

        self.pubSpeed.publish(Speed(self.speedLeft, self.speedRight, None, self.lastSpeedCmd[0], self.lastSpeedCmd[1]))
        self.inMotion = time.time()

    def readStep(self, msg):
        self.totLeftStep = msg.x
        self.totRightStep = msg.y

        self.pubStep.publish(Steps(self.totLeftStep, self.totRightStep, self.getHeader()))

    def readRange(self, msg):
        self.pubRange.publish(Distance(msg.field_of_view, msg.range, msg.radiation_type==1, self.getHeader()))      

    def readOrientation(self):
        while True:
            try:
                response = urllib.urlopen(self.URL)
                data=json.loads(response.read())
                i=data['rot_vector']['data'][-1]
                currentQuaternion=pyrr.quaternion.create(i[1][0],i[1][1],i[1][2],i[1][3])
                # self.heading = tf.transformations.euler_from_quaternion([i[1][0],i[1][1],i[1][2],i[1][3]])[2]*180/math.pi
                self.heading = Quaternion(currentQuaternion).to_euler123()[0]*180/math.pi
                print(self.heading)
                self.pubHeading.publish(self.heading)

            except Exception as e:
                print(e)
                print("Unable to connect to orientation sensor")
                time.sleep(3)
                continue

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
    phoneIP = rospy.get_param('~phoneIP')
    speed = rospy.get_param('~speed')
    pwm = rospy.get_param('~pwm')
    print(speed, pwm)
    VehicleAPI(phoneIP, speed, pwm)
    rospy.spin()

if __name__=='__main__':
    main()
