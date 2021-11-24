#!/usr/bin/env python3

from handlers.video import VideoHandler
import rospy
from std_msgs.msg import Bool

class LilyVision(object):

    def __init__(self):
        self.video_handler = VideoHandler()
        self.enabled = False

        rospy.init_node('video_listener')
        rospy.Subscriber("/video_enable", Bool, self.process_video_state)

        rospy.spin()
    
    def process_video_state(self, msg):
        resp = self.video_handler.enable(msg.data)
        print(resp)

if __name__ == '__main__':
    LilyVision()