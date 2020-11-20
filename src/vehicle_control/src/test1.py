# layer = 4
# a = []
# for k in range(layer):
#     c = []
#     l = k+1
#     for i in range(-l, l+1):
#         for j in [l, -l]:
#             c.append((i, j))
#             c.append((j, i))
#     a.extend(c)
#     print(len(set(c)))
# b = set(a)
# print(b)
# print(len(b))


#!/usr/bin/env python
# license removed for brevity
import rospy

# from std_msgs.msg import String
from vehicle_lib.msg import Distance, Speed, Location
from std_msgs.msg import Header
import numpy as np
import time

def talker():
    seqC = 0
    
    pub = rospy.Publisher('/pi/api/speed', Speed, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(20) # 10hz
    time.sleep(5)
    while not rospy.is_shutdown():
        a = np.random.normal(1.36, 0.15, 200)
        for i in range(len(a)):
            seqC += 1
            v = Header(seqC, None, '1')
            v.stamp = rospy.Time.now()
            b = Speed(a[i], a[i], v, a[i], a[i])
            rospy.loginfo(b)
            pub.publish(b)
            print(seqC)
            # rate.sleep()

        time.sleep(10)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException as e:
        print(e)