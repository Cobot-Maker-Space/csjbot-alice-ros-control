#!/usr/bin/env python

import asyncio
import websockets

async def test_socket(message):
    uri = "ws://localhost:60001"
    async with websockets.connect(uri) as websocket:
        await websocket.send(str(message))
        print(f"> {message}")

        response = await websocket.recv()
        print(f"< {response}")

test_payload = {
    "Msg_id": "NAVI_ROBOT_MOVE_REQ",
    "direction": 0,
}

asyncio.get_event_loop().run_until_complete(test_socket(test_payload))