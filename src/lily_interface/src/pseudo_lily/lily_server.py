#!/usr/bin/env python

import asyncio
import websockets
import ast

def process_message(message):
    payload = None
    message = ast.literal_eval(message)
    
    if message['Msg_id'] == 'NAVI_ROBOT_MOVE_REQ':
        payload = {
            "Msg_id": "NAVI_ROBOT_MOVE_RSP",
            "error_code": 0,
        }
    
    return payload

async def echo(websocket, path):
    async for message in websocket:
        print("REC:", message)
        response = process_message(message)
        print("RESP:", response)
        await websocket.send(str(response))

start_server = websockets.serve(echo, "localhost", 60001)

asyncio.get_event_loop().run_until_complete(start_server)
print("Pseudo Lily Service: Running....")
asyncio.get_event_loop().run_forever()