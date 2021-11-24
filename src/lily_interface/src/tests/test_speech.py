#!/usr/bin/env python3

import sys
import rospy
from std_msgs.msg import String


pub = rospy.Publisher('speech', String, queue_size=10)
rospy.init_node('talker', anonymous=True)
# rate = rospy.Rate(10) # 10hz

print("**** ENTERING DYNAMIC SPEACH MODE ****")
print('')

while(True):
    txt = input(":")
    if txt == 'q':
        sys.exit(1)
    pub.publish(txt)