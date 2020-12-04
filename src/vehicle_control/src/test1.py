# # layer = 4
# # a = []
# # for k in range(layer):
# #     c = []
# #     l = k+1
# #     for i in range(-l, l+1):
# #         for j in [l, -l]:
# #             c.append((i, j))
# #             c.append((j, i))
# #     a.extend(c)
# #     print(len(set(c)))
# # b = set(a)
# # print(b)
# # print(len(b))


# #!/usr/bin/env python
# # license removed for brevity
# import rospy

# # from std_msgs.msg import String
# from vehicle_lib.msg import Distance, Speed, Location, IntArr
# from vehicle_lib.srv import Explore, InitBound
# from std_msgs.msg import Header
# import numpy as np
# import time

# def talker():
#     seqC = 0
    
#     rospy.init_node("Explore")
#     initBoundSrv = rospy.ServiceProxy('/pi/exploration/initBound', InitBound)
#     nextPointSrv = rospy.ServiceProxy('/pi/exploration/nextPoint', Explore)

#     r = initBoundSrv(20,20,5,15,10,10)
#     a = np.zeros((20, 20))
#     l = []
#     for i in range(len(a)):
#         l.append(IntArr(a[i]))

#     n = (0, 0)
#     for k in range(5):
#         v = nextPointSrv(l, n[0], n[1]) 
#         # v.pts     
#         n = v.nextPoint.pts
#         print(n)


#     # pub = rospy.Publisher('/pi/api/speed', Speed, queue_size=10)
#     # rospy.init_node('talker', anonymous=True)
#     # rate = rospy.Rate(20) # 10hz
#     # time.sleep(5)
#     # while not rospy.is_shutdown():
#     #     a = np.random.normal(1.36, 0.15, 200)
#     #     for i in range(len(a)):
#     #         seqC += 1
#     #         v = Header(seqC, None, '1')
#     #         v.stamp = rospy.Time.now()
#     #         b = Speed(a[i], a[i], v, a[i], a[i])
#     #         rospy.loginfo(b)
#     #         pub.publish(b)
#     #         print(seqC)
#     #         # rate.sleep()

#     #     time.sleep(10)

# if __name__ == '__main__':
#     try:
#         talker()
#     except rospy.ROSInterruptException as e:
#         print(e)

# import math
# def convertRangeToPoint(distance, angle):
#         angleN = angle * math.pi / 180

#         if angleN > math.pi/2:
#             return round(-math.cos((math.pi) - angleN) * distance, 3), round(math.sin((math.pi)-angleN) * distance, 3)
        
#         return round(math.cos(angleN) * distance, 3), round(math.sin(angleN) * distance, 3)

# print(convertRangeToPoint(35, 33))
# print(convertRangeToPoint(35, 124))
# print(convertRangeToPoint(120, 33))
# print(convertRangeToPoint(120, 124))
# print(convertRangeToPoint(280, 33))
# print(convertRangeToPoint(280, 124))

import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style

import random
import rospy
from vehicle_lib.msg import Speed
import time
import threading

style.use('fivethirtyeight')

fig = plt.figure()
ax1 = fig.add_subplot(1,1,1)
ysL = []
ysR = []
xs = []
tPre = 0

def speedCallback(msg):
    ysL.append(float(msg.left))
    ysR.append(float(msg.right))
    xs.append(time.time())

def animate(i):
    global ysL, ysR, xs

    ax1.clear()
    ax1.plot(xs, ysL, linewidth=1)
    ax1.plot(xs, ysR, color='red', linewidth=1)
    # xsL = []
    # xsR = []
    # ys = []

def animAll():
    ani = animation.FuncAnimation(fig, animate, interval=100)
    plt.show()

def main():
    rospy.init_node("TestNode")
    
    tPre = time.time()
    # a = threading.Thread(target=animAll)
    c = rospy.Subscriber('/pi/api/speed', Speed, speedCallback)
    # a.start()
    animAll()
    rospy.spin()


main()
# if __name__=='__main__':
#     main()








from vehicle_lib.msg import Distance

# Test mapper

c = None

def streamScanResult():
    global c
    for i in range(0, 181, 2):
        if (i>=0 and i<=45) or (i>=135 and i<=180):
            # x = random.randint(65,75)
            x = random.randint(100,125)
        else:
            # continue
            x = random.randint(200, 250)

        if i >= 180:
            dis = Distance(i, x, True, None)
        else:
            dis = Distance(i, x, False, None)

        c.publish(dis)
        # time.sleep(0.1)

def main():
    global c
    rospy.init_node("TestNode")
    
    # tPre = time.time()
    # a = threading.Thread(target=animAll)
    c = rospy.Publisher('/pi/api/range', Distance, queue_size=10)
    # a.start()
    # animAll()
    while not rospy.is_shutdown():
        streamScanResult()
        time.sleep(3)
    # rospy.spin()


# main()

from tf import transformations as tra
import numpy as np
import math

import pyrr
# point - the point to be transformed
# traVec - translation vector
# rotAng - rotation angle to rotate the point around the z-axis
def transform(point, rotAng, traVec):
    point = (point[0], point[1], 0)
    traVec = (traVec[0], traVec[1], 0)

    mt = tra.translation_matrix(traVec)
    mr = tra.rotation_matrix(rotAng* math.pi/180, (0, 0, 1), (0,0,0))
    mat = mt.dot(mr)    # rotate then translate

    point = [point[0], point[1], 0, 1]

    return tuple(mat.dot(point)[:2])

# Positive angle is to +ve y-axis and -ve angle is to -ve y-axis
def transformFromXAxis(distance, rotAng, traVec=(0,0)):
    point = (distance, 0, 0)
    traVec = np.array((traVec[0], traVec[1], 0))

    p = pyrr.matrix33.create_from_z_rotation(rotAng * math.pi/180)
    rot = p.dot(point)
    return rot+traVec

# Positive angle is to +ve x-axis and -ve angle is to -ve x-axis
def transformFromYAxis(distance, rotAng, traVec=(0,0)):
    point = (0, distance, 0)
    traVec = np.array((traVec[0], traVec[1], 0))

    p = pyrr.matrix33.create_from_z_rotation(-rotAng * math.pi/180)
    rot = p.dot(point)
    return rot+traVec

# (transformFromXAxis(3,(2,0), 30))
# print(transformFromYAxis(3, 30, (2,0)))
# print(transform((1,0),110, (0,0)))