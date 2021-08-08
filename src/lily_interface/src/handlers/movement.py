#!/usr/bin/env python3

from handlers.base_handler import BaseHandler
import time

class MovementHandler(BaseHandler):
    def __init__(self):
        super().__init__()
        self.MSG_ID = "NAVI_ROBOT_MOVE_REQ"
        self.FORWARD = 0
        self.BACKWARD = 1
        self.LEFT = 2
        self.RIGHT = 3
        
    def _action(self, direction):
        payload = {
            "msg_id": self.MSG_ID,
            "direction": direction
        }
        return super()._action(payload)

    def move_left(self):
        return self._action(self.LEFT)

    def move_right(self):
        return self._action(self.RIGHT)

    def move_forward(self):
        return self._action(self.FORWARD)

    def move_back(self):
        return self._action(self.BACKWARD)

    # TODO - Handle error codes in response
