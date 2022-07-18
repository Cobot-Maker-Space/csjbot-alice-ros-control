#!/usr/bin/env python3

from handlers.base_handler import BaseHandler

class AuditoryHandler(BaseHandler):
    def __init__(self):
        super().__init__()
        self.MSG_ID_ENABLE = "SPEECH_ISR_START_REQ"
        self.MSG_ID_DISABLE = "SPEECH_ISR_STOP_REQ"
        self.MSG_ID_SETTINGS = "SPEECH_SET_ISR_PARAM_CMD"
        self.DEFAULT_LANG = "en_us"

    def _action(self, message_id):
        if (message_id == self.MSG_ID_SETTINGS) :
            payload = {
                "msg_id": message_id,
                "local_type": self.DEFAULT_LANG
            }

        else:
            payload = {
                "msg_id": message_id,
            }

        return super()._action(payload)

    def enable(self, state):
        if (state):
            self.set_language()
            msg = self.MSG_ID_ENABLE      
        else:
            msg = self.MSG_ID_DISABLE

        return self._action(msg)

    def set_language(self):  
        return self._action(self.MSG_ID_SETTINGS)

