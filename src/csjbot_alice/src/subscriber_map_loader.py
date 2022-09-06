#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

class MapLoader(object):

    def __init__(self):
        rospy.init_node('map_loader')
        rospy.Subscriber("/alice/context", String, self.switch_context)
        rospy.spin()

    def switch_context(self, msg):
       rospy.loginfo(f"Switching context to {msg}")

if __name__ == '__main__':
    MapLoader()
