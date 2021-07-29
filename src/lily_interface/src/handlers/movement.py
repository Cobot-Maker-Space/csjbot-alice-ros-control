#!/usr/bin/env python3

from handlers.web_socket import CommandHandler
import time

class MovementHandler(object):
    def __init__(self):
        self.MSG_ID = "NAVI_ROBOT_MOVE_REQ"
        self.FORWARD = 0
        self.BACKWARD = 1
        self.LEFT = 2
        self.RIGHT = 3
        self.commandHandler = CommandHandler()

    def _move(self, direction):
        payload = {
            "msg_id": self.MSG_ID,
            "direction": direction
        }
        return self.commandHandler.action(payload)

    def move_left(self):
        return self._move(self.LEFT)

    def move_right(self):
        return self._move(self.RIGHT)

    def move_forward(self):
        return self._move(self.FORWARD)

    def move_back(self):
        return self._move(self.BACKWARD)

    # TODO - Handle error codes in response

# if __name__ == "__main__":
#     rh = RobinMovement()
#
#     with raw_mode(sys.stdin):
#         while(True):
#             sys.stdin.read()
#

