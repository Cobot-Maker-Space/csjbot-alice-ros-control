import yaml
import asyncio
import os
import websockets
import json

class WebSocket(object):
    def __init__(self):
        self.load_config()

    def load_config(self):
        __location__ = os.path.realpath(os.path.join(os.getcwd(), os.path.dirname(__file__)))
        config_yaml = open(os.path.join(__location__, '../config/robin.yaml'))

        self.config_yaml = yaml.load(config_yaml, Loader=yaml.FullLoader)
        self.port = self.config_yaml['connection']['port']
        self.host = self.config_yaml['connection']['host']

    async def __aenter__(self):
        uri = f'ws://{self.host}:{self.port}'
        self._conn = websockets.connect(uri)
        self.websocket = await self._conn.__aenter__()
        return self

    async def __aexit__(self, *args, **kwargs):
        await self._conn.__aexit__(*args, **kwargs)

    async def send(self, message):
        await self.websocket.send(message)

    async def receive(self):
        return await self.websocket.recv()

class CommandHandler(object):
    def __init__(self):
        self.wws = WebSocket()
        self.loop = asyncio.get_event_loop()

    def action(self, payload):
        return self.loop.run_until_complete(
            self.__async__get_ticks(payload)
        )

    async def __async__get_ticks(self, payload):
        async with self.wws as echo:
            await echo.send(json.dumps(payload))
            return await echo.receive()


if __name__ == '__main__':
    ch = CommandHandler()

    # Test command
    payload_right = {
        "msg_id": "NAVI_ROBOT_MOVE_REQ",
        "direction": 3  # RIGHT
    }
    res = ch.action(payload_right)
    print(res)

