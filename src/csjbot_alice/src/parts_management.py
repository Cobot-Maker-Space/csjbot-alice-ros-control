#!/usr/bin/env python3

import rospy
from csjbot_alice.msg import PartTransfer, PartsListUpdate

class PartsManagement(object):

    def __init__(self):
        rospy.init_node('part_transfer_listener')
        rospy.Subscriber("/alice/parts/transfer/", PartTransfer, self.process_parts_transfer)
        
        self.pub_updated_parts_list = rospy.Publisher("/alice/parts/updated/", PartsListUpdate, queue_size=10, latch=False)
        
        rospy.spin()
    
    def process_parts_transfer(self, msg):
        parts_source = self.get_parts_list(msg.location_source)
        parts_target = self.get_parts_list(msg.location_target)

        part_exists = False
        for source_part in parts_source:
            if source_part['id'] == msg.id:
                rospy.loginfo(f"matched part id in source {msg.id}")
                part_details = source_part.copy()
                source_part['qty'] = source_part['qty'] - 1   
                    
                if source_part['qty'] == 0:
                    parts_source.remove(source_part)

                for target_part in parts_target:
                    if target_part['id'] == msg.id:
                        rospy.loginfo(f"existing part id in target {msg.id}")
                        # rospy.loginfo(f"Original qty {target_part['qty']}")
                        target_part['qty'] += 1
                        # rospy.loginfo(f"New qty {target_part['qty']}")
                        part_exists = True

                if not part_exists:
                    rospy.loginfo("appending new part")
                    part_details['qty'] = 1
                    parts_target.append(part_details)

        
        # print(parts_source)
        self.set_parts_list(msg.location_source, parts_source)
        self.set_parts_list(msg.location_target, parts_target)

    def get_parts_list(self, location):
        parts_list_by_location = rospy.get_param("/alice/parts/" + location, None)
        if parts_list_by_location is None:
            parts_list_by_location = []

        return parts_list_by_location

    def set_parts_list(self, location, parts):
        rospy.set_param("/alice/parts/" + location, parts)
        self.pub_updated_parts_list.publish(PartsListUpdate(location))

if __name__ == '__main__':
    PartsManagement()