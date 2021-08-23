#!/usr/bin/env python3

from handlers.web_socket import CommandHandler
import time

class BaseHandler(object):

    def __init__(self):
        self.MSG_ID = ''
        self.commandHandler = CommandHandler()

    def _action(self, payload, await_for_return=True):
        return self.commandHandler.action(payload, await_for_return)
