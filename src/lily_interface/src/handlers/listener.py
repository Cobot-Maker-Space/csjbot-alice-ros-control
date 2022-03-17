#!/usr/bin/env python3

from config.loader import ConfigLoader 
import asyncio
import websockets
import io, sys
import json
import rospy
import datetime

class SocketListenerHandler(object):
    def __init__(self):
        self.APP_ENV = 'prod'
        self.config_loader = ConfigLoader(self.APP_ENV)
        self.port = self.config_loader.fetch_value('port')
        self.host = self.config_loader.fetch_value('host')

    def start_listening(self):
        uri = f'ws://{self.host}:{self.port}'
        asyncio.get_event_loop().run_until_complete(self.listen(uri))

    async def listen(self, uri):
        rospy.loginfo(f"Starting to listen on port {self.port}")
        try:
            async with websockets.connect(uri) as websocket:
                while rospy.spin():
                    self.handle_data(await websocket.recv())
            
        except asyncio.exceptions.TimeoutError as e:
            rospy.logerr('Error connecting to web sockets. Timeout exceeded. Check the status of Alice.')
            sys.exit(-1)
        
    
    def handle_data(self, data):
        payload = json.loads(data)
        msg_id = payload['msg_id']
        rospy.loginfo(payload)        
 
if __name__ == "__main__":
    slh = SocketListenerHandler()
    slh.start_listening()