#!/usr/bin/env python3

from handlers.listener import SocketListenerHandler
import rospy 

class LilySocketListener(object):

    def __init__(self):

        rospy.init_node('alice_listener')

        self.socket_listener = SocketListenerHandler()
        self.socket_listener.start_listening()
    
if __name__ == '__main__':
<<<<<<< HEAD
    LilySocketListener()
=======
    LilySocketListener()
>>>>>>> 6112b5d6f0768871a76da82e27e08237d995c95d
