#!/usr/bin/env python3

from handlers.base_handler import BaseHandler

class VideoHandler(BaseHandler):
    def __init__(self):
        super().__init__()
        self.MSG_ID_ENABLE = "FACE_DETECT_OPEN_VIDEO_REQ"
        self.MSG_ID_DISABLE = "FACE_DETECT_CLOSE_VIDEO_REQ"

    def _action(self, msg_id):
        payload = {
            "msg_id": msg_id,
        }
        return super()._action(payload)

    def enable(self, state):        
        msg_id = self.MSG_ID_ENABLE if state else self.MSG_ID_DISABLE
        return self._action(msg_id)

