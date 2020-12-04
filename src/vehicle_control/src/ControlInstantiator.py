#!/usr/bin/env python

import rospy
from vehicle_lib.msg import Location, Speed
from vehicle_lib.srv import Goto, Travel

from std_msgs.msg import Float32, Header

from InitialRotatePID import controller
import threading

class ControlInstantiator():
    def __init__(self):
        self.queuedPositions=[]
        self.currentPosition=[0.0,0.0]
        self.currentOrientation=0.0

        self.pubCurLoc = rospy.Subscriber('/pi/localization/currentLoc', Location, self.curLocCallback)

        self.travelSrv = rospy.Service('/pi/steering/travel', Travel, self.travelService)
        self.gotoSrv = rospy.ServiceProxy('/pi/steering/goto', Goto)

    def addToQueue(self,msg):
        #Do something here to add
        pass

    def travelService(self, msg):
        p = msg.path.path
        for i in p:
            self.queuedPositions.append((i.x, i.y))

        self.instantiator()

    def curLocCallback(self, msg):
        self.currentPosition = [msg.x, msg.y]
        
    def instantiator(self):
        for i in self.queuedPositions:
            reached = self.gotoSrv(self.currentPosition, i)

            if not reached:
                return False

        return True

def main():
    rospy.init_node("ControlInstantiator")
    ControlInstantiator()
    rospy.spin()

if __name__=='__main__':
    main()