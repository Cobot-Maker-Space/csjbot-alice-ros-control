#!/usr/bin/env python

import asyncio
import websockets
import ast
import time

def process_message(message):
    payload = None
    message = ast.literal_eval(message)
    
    if message['msg_id'] == 'NAVI_ROBOT_MOVE_REQ':
        time.sleep(5)
        payload = {
            "msg_id": "NAVI_ROBOT_MOVE_RSP",

            "error_code": 0,
        }
    
    return payload

async def echo(websocket, path):
    async for message in websocket:
        print("REC:", message)
        response = process_message(message)
        print("RESP:", response)
        await websocket.send(str(response))

start_server = websockets.serve(echo, '172.17.0.2', 60001)

asyncio.get_event_loop().run_until_complete(start_server)
print("Pseudo Lily Service: Running....")
asyncio.get_event_loop().run_forever()