#!/usr/bin/env python3

from pickle import STRING
from handlers.base_handler import BaseHandler
from config.loader import ConfigLoader
import asyncio
import io
import PIL.Image
import PIL.ImageOps as ImageOps
import numpy as np

import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String

class VideoOptionsHandler(BaseHandler):
    def __init__(self):
        super().__init__()
        self.MSG_ID_ENABLE = "FACE_DETECT_OPEN_VIDEO_REQ"
        self.MSG_ID_DISABLE = "FACE_DETECT_CLOSE_VIDEO_REQ"

    def _action(self, msg_id):
        payload = {
            "msg_id": msg_id,
        }
        return super()._action(payload)

    def enable(self, state):
        msg_id = self.MSG_ID_ENABLE if state else self.MSG_ID_DISABLE
        return self._action(msg_id)


class VideoHandler(BaseHandler):
    def __init__(self):
        super().__init__()
        self.APP_ENV = rospy.get_param('/APP_ENV', 'dev')
        video_config_loader = ConfigLoader(self.APP_ENV)
        video_config_loader.set_type('video_connection')
        video_config_loader.load_config()

        self.video_port = video_config_loader.fetch_value('port')
        self.video_host = video_config_loader.fetch_value('host')

        self.current_image = None
        self.bridge = CvBridge()

        rospy.init_node('robin_vision')
        self.image_publisher = rospy.Publisher("/camera/image", Image, queue_size=10)
        self.image_num_publisher = rospy.Publisher("/camera/image/number", String, queue_size=10)

        # self.loop_rate = rospy.Rate(1)

    def launch_video(self):
        asyncio.run(self.receive_stream())

    async def receive_stream(self):
        rospy.loginfo("Starting video stream")
        reader, writer = await asyncio.open_connection(self.video_host, self.video_port)

        header_bytes = bytearray(b'\xff\xfe\xfd\xfc\xfb\xfa\xd8')
        footer_bytes = bytearray(b'\xff\xfe\xfd\xfc\xfb\xfa\xd9\x00')
        jpeg_header = bytearray(b'\xff\xd8\xff')

        data = b""

        start_pos = -1
        end_pos = -1

        i = 0

        while not rospy.is_shutdown():
            data_stream = await reader.read(4096)
            i += 1

            if not data_stream:
                break

            data += data_stream

            if start_pos == -1:
                start_pos = data.find(header_bytes)

            if start_pos != -1:
                end_pos = data.find(footer_bytes, start_pos)
                if end_pos != -1:
                    jpeg_header_pos = data.find(jpeg_header, start_pos)

                    image_bytearr = bytearray(data)
                    image_data = bytes(image_bytearr[jpeg_header_pos:end_pos])

                    dataBytesIO = io.BytesIO(image_data)
                    pil_img = PIL.Image.open(dataBytesIO).convert('RGB')
                    pil_img = ImageOps.mirror(pil_img)

                    cv2_img = np.array(pil_img)
                    cv2_img = cv2_img[:, :, ::-1].copy()

                    self.current_image = cv2_img

                    img_msg = self.bridge.cv2_to_imgmsg(cv2_img, encoding="bgr8")
                    self.image_publisher.publish(img_msg)

                    start_pos = -1
                    end_pos = -1
                    data = b""

        rospy.loginfo('Close the connection')
        writer.close()
        await writer.wait_closed()

    def fetch_image(self):
        return self.current_image


