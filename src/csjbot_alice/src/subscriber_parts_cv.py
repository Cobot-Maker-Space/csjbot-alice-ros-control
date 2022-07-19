#!/usr/bin/env python3

from handlers.video import VideoOptionsHandler
import rospy
import cv2
from cv_bridge import CvBridge
from std_msgs.msg import Empty
from sensor_msgs.msg import Image

class LilyPartsRecognition(object):

    def __init__(self):
      
        rospy.init_node('parts_recognition')
        rospy.Subscriber("/alice/parts/scan", Empty, self.process_image)
        rospy.spin()
    
    def process_image(self, msg):
        image_msg = rospy.wait_for_message("/camera/image")

        bridge = CvBridge()
        orig = bridge.imgmsg_to_cv2(image_msg, "bgr8")
        cv2.imwrite("parts.jpg", orig)
       
if __name__ == '__main__':
    LilyPartsRecognition()