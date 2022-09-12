#!/usr/bin/env python3

import pathlib

import rospy
import rospkg
from std_msgs.msg import String
from slamware_ros_sdk.srv import SyncSetStcm


class MapLoader(object):
    def __init__(self):
        rospy.init_node("map_loader")
        rospy.wait_for_service("/slamware_ros_sdk_server_node/sync_set_stcm")
        self.load_map_service = rospy.ServiceProxy(
            "/slamware_ros_sdk_server_node/sync_set_stcm", SyncSetStcm
        )
        rospy.Subscriber("/alice/load_map", String, self.load_map)
        rospy.spin()

    def load_map(self, msg):
        rospack = rospkg.RosPack()
        bdir = rospack.get_path("csjbot_alice")
        try:
            fname = rospy.get_param(f"/alice/maps/{msg.data}/filename")
            p = pathlib.Path(f"{bdir}/config/maps/{fname}")
            if p.exists() and p.is_file():
                rospy.loginfo(f"Switching map to {msg.data}")
                with p.open(mode="rb") as f:
                    self.load_map_service(f.read(), None)
            else:
                rospy.logwarn(f"Unable to load map: {msg.data}")
        except KeyError:
            rospy.logwarn(f"No such map: {msg.data}")


if __name__ == "__main__":
    MapLoader()
