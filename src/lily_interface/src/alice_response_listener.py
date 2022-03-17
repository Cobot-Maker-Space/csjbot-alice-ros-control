#!/usr/bin/env python3

from handlers.listener import SocketListenerHandler
import rospy 

class LilySocketListener(object):

    def __init__(self):

        rospy.init_node('alice_listener')

        self.socket_listener = SocketListenerHandler()
        while not rospy.is_shutdown():
            self.socket_listener.start_listening()
    
if __name__ == '__main__':
    LilySocketListener()
