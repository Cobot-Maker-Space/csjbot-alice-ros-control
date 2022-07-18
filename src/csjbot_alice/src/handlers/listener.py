#!/usr/bin/env python3

from config.loader import ConfigLoader
import asyncio
import websockets
import io, sys
import json
import rospy
import datetime
from std_msgs.msg import String, Bool

class SocketListenerHandler(object):
    def __init__(self):
        self.APP_ENV = 'prod'
        self.config_loader = ConfigLoader(self.APP_ENV)
        self.port = self.config_loader.fetch_value('port')
        self.host = self.config_loader.fetch_value('host')

        self.pub_video_state = rospy.Publisher('/video_on', Bool, queue_size=10, latch=True)
        self.pub_video_state = rospy.Publisher('/auditory/message', String, queue_size=10, latch=True)

        self.FACE_DETECT_OPEN_RESPONSE = "FACE_DETECT_OPEN_VIDEO_RSP"
        self.FACE_DETECT_CLOSE_RESPONSE = "FACE_DETECT_OPEN_VIDEO_RSP"
        self.SPEECH_DETECTION = "SPEECH_ISR_ONLY_RESULT_NTF"

    def start_listening(self):
        uri = f'ws://{self.host}:{self.port}'
        asyncio.get_event_loop().run_until_complete(self.listen(uri))

    async def listen(self, uri):
        rospy.loginfo(f"Starting to listen on port {self.port}")
        try:
            async with websockets.connect(uri) as websocket:
                while True:
                    self.handle_data(await websocket.recv())

        except asyncio.exceptions.TimeoutError as e:
            rospy.logerr('Error connecting to web sockets. Timeout exceeded. Check the status of Alice.')
            sys.exit(-1)

    def handle_data(self, data):
        payload = json.loads(data)
        msg_id = payload['msg_id']

        if msg_id == self.FACE_DETECT_OPEN_RESPONSE:
            # face detect on 
            self.pub_video_state.publish(Bool(True))
            # TODO - WHAT IS GOING ON HERE???

        if msg_id == self.FACE_DETECT_CLOSE_RESPONSE:
            # face detect on
            self.pub_video_state.publish(Bool(False))
            # TODO - WHAT IS GOING ON HERE???

        if msg_id == self.SPEECH_DETECTION:
            # speech detection
            self.pub_auditory.publish(payload['text'])

        rospy.loginfo(payload)

if __name__ == "__main__":
    slh = SocketListenerHandler()
    slh.start_listening()
