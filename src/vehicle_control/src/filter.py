#!/usr/bin/env python

import numpy as np
import rospy
from std_msgs.msg import Float32, Int16, Header, Time, Empty, Int32
from sensor_msgs.msg import Range
from geometry_msgs.msg import Point
from vehicle_lib.msg import Distance, Speed, Steps
from quaternion import Quaternion
import pyrr, urllib, json, time, math, threading

class Filter:
    def __init__(self):
        self.lastTime = 0
        self.rollingWindow = 5
        self.speedLeft = []
        self.speedRight = []
        # self.avgErr

        # self.subSpeed = rospy.Subscriber('/pi/api/speed', Speed, self.recordSpeed)


    def recordSpeed(self, msg):
        self.speedLeft.append(msg.left)
        self.speedRight.append(msg.right)
        s = self.speedLeft[len(self.speedLeft)-self.rollingWindow:]
        q = self.speedRight[len(self.speedRight)-self.rollingWindow:]
        s.sort()
        q.sort()

        # if len(self.speedLeft) > 3:
        # print(s)
        # while s[-1] - s[0] <= 0.2: 
        if s[-1] - s[0] > 0.1:
            if s[-2] - s[0] > 0.1:
                s = s[1:]
            else:
                s = s[:-1]

        # while q[-1] - q[0] <= 0.2: 
        if q[-1] - q[0] > 0.1:
            if q[-2] - q[0] > 0.1:
                q = q[1:]
            else:
                q = q[:-1]

        print(np.array(s).mean(), np.array(q).mean())


    def rollingAverage(self, speed):
        if len(speed)>0:
            s = speed[-1]
            c = np.array(s[len(s)-self.rollingWindow:])
            return c.mean()

        return 0

    def generateSampleData(self, mean, sd, qty):
        return np.random.normal(mean, sd, qty)

    def filter(self, data, preK=0.8, postK=0.2):
        res = []
        pre = data[0]
        err = 0
        for i in data[1:]:
            tempPre = pre
            pre = round(pre * preK + i * postK, 4)
            res.append(pre)
            err += abs(tempPre - i)

        return res, err


def main():
    # rospy.init_node("Filter")
    x = Filter()
    # preK = postK = hisPreK = tempPreK = 0.5
    # preErr = None
    # factor = 0.035
    # tolerance = 1
    a = x.generateSampleData(1.36, 0.2, 500)
    # y  = None

    # for i in range(10000):
    #     y, err = x.filter(a, preK, postK)

    #     if err <= tolerance:
    #         print(np.array(y))
    #         print(preK, postK, err, i)
    #         return

    #     if preErr == None:
    #         print(err)
    #         preErr = err
        
    #     else:
    #         tempPreK = preK
    #         if preErr < err:
    #             if hisPreK < preK:
    #                 preK = hisPreK - hisPreK * factor
                
    #             else:
    #                 preK = hisPreK + hisPreK * factor

    #         else:
    #             if hisPreK < preK:
    #                 preK += preK * factor
                
    #             else:
    #                 preK -= preK * factor

    #         hisPreK = tempPreK
    #         postK = 1 - preK
    # print(preK, postK, err, i)

    # print(err)
    # print(len(a), len(y))
    # for k in range(len(a)-1):
    #     print(a[k+1], round(y[k], 4))
    
    y, err = x.filter(a, 0.85, 0.15)

    import matplotlib.pyplot as pyplot
    pyplot.plot(range(1, len(a)+1), a)
    pyplot.plot(range(len(y)), y)
    pyplot.show()

    
    # print(err)
    # rospy.spin()

if __name__=='__main__':
    main()


# 0.8679377092193401, 0.1320622907806599