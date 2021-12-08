#!/usr/bin/env python3

from handlers.video import VideoOptionsHandler
import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class LilyTestVision(object):

    def __init__(self):
      
        rospy.init_node('image_listener')
        rospy.Subscriber("/camera/image", Image, self.process_image)
        rospy.spin()
    
    def process_image(self, msg):
        bridge = CvBridge()
        orig = bridge.imgmsg_to_cv2(msg, "bgr8")
        cv2.imwrite("test2.jpg", orig)
       
if __name__ == '__main__':
    LilyTestVision()