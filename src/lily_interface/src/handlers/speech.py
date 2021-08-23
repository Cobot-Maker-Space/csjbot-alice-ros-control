#!/usr/bin/env python3

from handlers.base_handler import BaseHandler

class SpeechHandler(BaseHandler):
    def __init__(self):
        super().__init__()
        self.MSG_ID = "SPEECH_TTS_REQ"
        self.MSG_ID_SETTINGS = "SPEECH_SET_TTS_PARAM_CMD"

    def _action(self, message):
        payload = {
            "msg_id": self.MSG_ID,
            "content": message
        }
        return super()._action(payload)

    def speak(self, message):
        return self._action(message.data)

    def update_settings(self, 
            voice="xiaoyan", 
            pitch=50, 
            speed=50,
            rnd=1       # 1 = One-hundred and 23; 2 = 123;
            ):  

        payload = {
            "msg_id": self.MSG_ID_SETTINGS, 
            "engine_type": 0, 
            "voice_name": voice, 
            "speed": speed,
            "Pitch": pitch, 
            "rdn": rnd
        }

        return super()._action(payload, await_for_return=False)

