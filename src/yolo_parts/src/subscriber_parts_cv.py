#!/usr/bin/env python3

from socket import timeout
# from handlers.video import VideoOptionsHandler
import rospy
import cv2
from yolo_parts import YoloParts
from cv_bridge import CvBridge
from std_msgs.msg import Empty
from sensor_msgs.msg import Image
import os

class LilyPartsRecognition(object):

    def __init__(self):
        
        self.yolo = YoloParts(weights='weights/hri_parts_best_5s_phase2.pt', data='data/lego.yaml')
        self.image_store = os.path.realpath(os.path.join(os.getcwd(), os.path.dirname(__file__))) + "/image_store/"

        rospy.init_node('parts_recognition')
        rospy.Subscriber("/alice/parts/scan", Empty, self.process_image)
        rospy.spin()
    
    def process_image(self, msg):
        # image_msg = rospy.wait_for_message("/camera/image", Image, timeout=5)

        # bridge = CvBridge()
        # original_image = bridge.imgmsg_to_cv2(image_msg, "bgr8")
        # cv2.imwrite(self.image_store + "parts_on_tray.jpg", original_image)

        # Run yolo model on the image and retrieve the parts list
        result = self.yolo.detect_image(self.image_store + "parts_on_tray.jpg")
        #LOOP THROUGH PARTS AND PUBLISH THEM TO CORRECT TOPIC FOR UI
        for part in result:
            print(f"{part['id']}: {part['conf']}")

        # rospy.loginfo(result)

if __name__ == '__main__':
    LilyPartsRecognition()