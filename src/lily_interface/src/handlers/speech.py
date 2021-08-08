#!/usr/bin/env python3

from handlers.base_handler import BaseHandler

class SpeechHandler(BaseHandler):
    def __init__(self):
        super().__init__()
        self.MSG_ID = "SPEECH_TTS_REQ"

    def _action(self, message):
        payload = {
            "msg_id": self.MSG_ID,
            "content": message
        }
        return super()._action(payload)

    def speak(self, message):
        return self._action(message.data)