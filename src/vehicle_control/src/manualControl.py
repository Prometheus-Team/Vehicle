#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32, Int16, Header, Time, Empty, Int32
from sensor_msgs.msg import Range
from vehicle_lib.msg import Distance, Speed, Steps
from quaternion import Quaternion
import pyrr, urllib, json, time, math, threading

from pynput.keyboard import Listener, Key

import turtle
class ManualControl:
    def __init__(self):
        self.seqC = 0
        self.constSpeed = 1.5
        self.up = self.down = self.left = self.right = False
        
        self.pubSpeed = rospy.Publisher('/pi/api/speedCmd', Speed, queue_size=10)
        
        # 0 - stop      1 - forward    2 - backward     3 - left        4 - right
        self.subManual = rospy.Subscriber('/pi/api/manual', Int16, queue_size=10)   


        with Listener(on_press=self.onPress, on_release=self.onRelease) as listener:
            # listener.setDaemon(True)
            listener.join()

    # steer the vehicle with given angle in the given direction
    def turn(self, direction):
        speedLeft = speedRight = self.constSpeed

        if direction == 'Left':
            speedLeft *= -1

        elif direction == 'Right':
            speedRight *= -1

        else:
            speedLeft = speedRight = 0

        s = Speed(left=speedLeft, right=speedRight, header=self.getHeader())
        self.pubSpeed.publish(s)

    # move dorward or backward with given speed
    def driveStraight(self, speed, direction):
        speedLeft = speedRight = speed
        if direction == 'Backward':
            speedLeft *= -1
            speedRight *= -1

        elif direction == 'Left':
            speedRight *= 1.1

        elif direction == 'Right':
            speedLeft *= 1.1

        s = Speed(left=speedLeft, right=speedRight, header=self.getHeader())
        print("Called straight drive")
        self.pubSpeed.publish(s)

    def onPress(self, key):
        # if self.up or self.down or self.left or self.right:
        #     return

        if key == Key.up and not self.up:
            self.up = True
            self.driveStraight(self.constSpeed, "Forward")
            time.sleep(0.05)
            self.driveStraight(self.constSpeed, "Forward")
            # time.sleep(0.5)

        elif key == Key.down and not self.down:
            self.down = True
            self.driveStraight(self.constSpeed, "Backward")
            time.sleep(0.05)
            self.driveStraight(self.constSpeed, "Backward")
            # time.sleep(0.5)

            
        elif key == Key.left and not self.left:
            self.left = True
            self.turn("Left")
            # time.sleep(0.5)

            
        elif key == Key.right and not self.right:
            self.right = True
            self.turn("Right")
            # time.sleep(0.5)


    def onRelease(self, key):
        if key == Key.up or key == Key.down:
            self.up = False
            self.down = False
            self.driveStraight(0, "Forward")
            print("Not driving")

        elif key == Key.left or key == Key.right:
            self.left = False
            self.right = False
            self.turn("No")
            print('Not turnning')

    def getHeader(self):
        self.seqC += 1
        v = Header(self.seqC, None, '1')
        v.stamp = rospy.Time.now()
        return v


def main():
    rospy.init_node("ManualControlAPI")
    ManualControl()
    rospy.spin()

if __name__=='__main__':
    main()