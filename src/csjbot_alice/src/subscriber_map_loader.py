#!/usr/bin/env python3

import pathlib

import rospy
import rospkg
from std_msgs.msg import String
from slamware_ros_sdk.msg import ClearMapRequest, MapKind
from slamware_ros_sdk.srv import SyncSetStcm


class MapLoader(object):
    def __init__(self):
        rospy.init_node("map_loader")
        self.clear_map_publisher = rospy.Publisher(
            "/slamware_ros_sdk_server_node/clear_map", ClearMapRequest, queue_size=10
        )
        rospy.wait_for_service("/slamware_ros_sdk_server_node/sync_set_stcm")
        self.load_map_service = rospy.ServiceProxy("/slamware_ros_sdk_server_node/sync_set_stcm", SyncSetStcm)
        rospy.Subscriber("/alice/context", String, self.switch_context)
        rospy.spin()

    def switch_context(self, msg):
        rospy.loginfo(f"Switching context to {msg.data}")
        rospack = rospkg.RosPack()
        bdir = rospack.get_path("csjbot_alice")
        p = pathlib.Path(f"{bdir}/config/maps/{msg.data}.stcm")
        if p.exists() and p.is_file():
          with p.open(mode="rb") as f:
            self.load_map_service(f.read(), None)
        else:
            req = ClearMapRequest()
            self.clear_map_publisher.publish(req)


if __name__ == "__main__":
    MapLoader()
