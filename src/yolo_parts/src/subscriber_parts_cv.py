#!/usr/bin/env python3

from socket import timeout
# from handlers.video import VideoOptionsHandler
import rospy
import cv2
from yolo_parts import YoloParts
from cv_bridge import CvBridge
from std_msgs.msg import Empty
from sensor_msgs.msg import Image
from csjbot_alice.msg import PartTransfer, PartsListUpdate

import os

class LilyPartsRecognition(object):

    def __init__(self):
        self.yolo = YoloParts(weights='weights/hri_parts_best_5s_phase2.pt', data='data/lego.yaml')
        self.image_store = os.path.realpath(os.path.join(os.getcwd(), os.path.dirname(__file__))) + "/image_store/"

        rospy.init_node('parts_recognition')
        rospy.Subscriber("/alice/parts/scan", Empty, self.process_image)

        # self.pub_intransit_parts_list = rospy.Publisher("/alice/parts/intransit", PartTransfer, queue_size=10, latch=False)
        self.pub_updated_parts_list = rospy.Publisher("/alice/parts/updated/", PartsListUpdate, queue_size=10, latch=False)
        self.pub_parts_transfer = rospy.Publisher("/alice/parts/transfer/", PartTransfer, queue_size=10, latch=False)
       
        rospy.spin()
    
    def process_image(self, msg):
        image_msg = rospy.wait_for_message("/camera/image", Image, timeout=5)

        bridge = CvBridge()
        original_image = bridge.imgmsg_to_cv2(image_msg, "bgr8")
        cv2.imwrite(self.image_store + "parts_on_tray.jpg", original_image)

        # Get existing list of tray parts and move back to warehouse
        tray_parts = self.get_parts_by_location('intransit')
        
        if tray_parts is not None:
            for part in tray_parts:
                self.transfer_part(part['id'], 'intransit', 'warehouse')

        # Run yolo model on the image and retrieve the parts list
        result = self.yolo.detect_image(self.image_store + "parts_on_tray.jpg")

        for part in result:
            part_id = self.part_id_from_code(part['id'])
            if part_id is not None:
                self.transfer_part(part_id, 'warehouse', 'intransit')
                
        self.pub_updated_parts_list.publish(PartsListUpdate('warehouse'))
        self.pub_updated_parts_list.publish(PartsListUpdate('intransit'))

    def transfer_part(self, id, source, target):
        part_transfer = PartTransfer()
        part_transfer.id = id
        part_transfer.location_source = source
        part_transfer.location_target = target
        part_transfer.trigger_update = False
        self.pub_parts_transfer.publish(part_transfer)       
    
    def part_id_from_code(self, code):
        part_id = None

        parts_locations = rospy.get_param("/alice/parts/", None)
        for location in parts_locations:
            parts_by_location = self.get_parts_by_location(location)
            for part in parts_by_location: 
                if part['code'] == code:
                    part_id = part['id']
                    break

        return part_id

    def get_parts_by_location(self, location):
        parts = rospy.get_param("/alice/parts/" + location, None)
        return parts
        
if __name__ == '__main__':
    LilyPartsRecognition()
