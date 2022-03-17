import yaml
import asyncio
import os
import websockets
import json
import sys
import rospy
from config.loader import ConfigLoader 

class WebSocket(object):
    def __init__(self):
        self.TIMEOUT = 5
        # self.APP_ENV = rospy.get_param('APP_ENV', 'dev')
        self.APP_ENV = 'prod'
        
        self.config_loader = ConfigLoader(self.APP_ENV)
        self.port = self.config_loader.fetch_value('port')
        self.host = self.config_loader.fetch_value('host')

    async def __aenter__(self):
        uri = f'ws://{self.host}:{self.port}'
        try:
            # self._conn = await asyncio.wait_for(websockets.connect(uri), self.TIMEOUT)
            # self.websocket = await self._conn.__aenter__()
            self._conn = websockets.connect(uri)
            self.websocket = await self._conn.__aenter__()

        except asyncio.exceptions.TimeoutError as e:
            rospy.logerr('Error connecting to web sockets. Timeout exceeded. Check the status of Lily.')
            sys.exit(-1)

        return self

    async def __aexit__(self, *args, **kwargs):
        try: 
            await self._conn.__aexit__(*args, **kwargs)
        except:
            rospy.logerr('Error closing sockets')
            pass

    async def send(self, message):
        await self.websocket.send(message)

    async def receive(self):
        return await self.websocket.recv()

class CommandHandler(object):
    def __init__(self):
        self.wws = WebSocket()
        self.loop = asyncio.get_event_loop()

    def action(self, payload, wait_for_return=False):  
        return self.loop.run_until_complete(
            self.__async__get_ticks(payload, wait_for_return)
        )

    async def __async__get_ticks(self, payload, wait_for_return):
        async with self.wws as echo:
            await echo.send(json.dumps(payload))
           
if __name__ == '__main__':
    ch = CommandHandler()

    # Test command
    payload_right = {
        "msg_id": "NAVI_ROBOT_MOVE_REQ",
        "direction": 3  # RIGHT
    }
    res = ch.action(payload_right)
    print(res)

