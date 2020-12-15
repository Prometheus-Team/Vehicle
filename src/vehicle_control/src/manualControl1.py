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
        self.constSpeed = 2
        self.up = self.down = self.left = self.right = False
        
        self.pubSpeed = rospy.Publisher('/pi/api/speedCmd', Speed, queue_size=10)
        
        # 0 - stop      1 - forward    2 - backward     3 - left        4 - right
        self.subManual = rospy.Subscriber('/pi/api/manual', Int16, self.manualListener)   

    def manualListener(self, msg):
        if msg.data == 0:
            self.driveStraight(0, "Forward")

        elif msg.data == 1:
            self.driveStraight(self.constSpeed, "Forward")

        elif msg.data == 2:
            self.driveStraight(self.constSpeed, "Backward")

        elif msg.data == 3:
            self.turn("Left")

        elif msg.data == 4:
            self.turn("Right")


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

        s = Speed(left=speedLeft, right=speedRight, header=self.getHeader())
        print("Called straight drive")
        self.pubSpeed.publish(s)

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