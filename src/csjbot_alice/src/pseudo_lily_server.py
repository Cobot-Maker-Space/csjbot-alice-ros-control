#!/usr/bin/env python

import asyncio
import websockets
import ast
import time
import os
import yaml
from config.loader import ConfigLoader 

def process_message(message):
    payload = None
    message = ast.literal_eval(message)
    
    if message['msg_id'] == 'NAVI_ROBOT_MOVE_REQ':
        time.sleep(2)
        payload = {
            "msg_id": "NAVI_ROBOT_MOVE_RSP",
            "error_code": 0,
        }

    if message['msg_id'] == 'SPEECH_TTS_REQ':
        payload = {
            "msg_id": "SPEECH_TTS_RSP",
            "error_code": 0,
        }
    
    if message['msg_id'] == 'SET_ROBOT_EXPRESSION_REQ':
        payload = {
            "msg_id": "SET_ROBOT_EXPRESSION_RSP",
            "expression": message['expression'],
            "error_code": 0,
        }

    return payload

async def echo(websocket, path):
    async for message in websocket:
        print("REC:", message)
        response = process_message(message)
        print("RESP:", response)
        await websocket.send(str(response))

SERVER_MODE = 'dev'
config_loader = ConfigLoader(SERVER_MODE)
start_server = websockets.serve(echo, config_loader.fetch_value('host'), config_loader.fetch_value('port'))

asyncio.get_event_loop().run_until_complete(start_server)
print("Pseudo Lily Service: Running....")
asyncio.get_event_loop().run_forever()