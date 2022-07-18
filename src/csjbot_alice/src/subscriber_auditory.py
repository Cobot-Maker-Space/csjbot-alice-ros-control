#!/usr/bin/env python3

from handlers.auditory import AuditoryHandler
import rospy
from std_msgs.msg import String, Bool
from csjbot_alice.msg import SpeechSettings

class LilyAuditory(object):

    def __init__(self):
        self.auditory_handler = AuditoryHandler()

        rospy.init_node('auditory_listener')
        rospy.Subscriber("/auditory/enable", Bool, self.process_auditory_state)

        rospy.spin()
    
    def process_auditory_state(self, msg):
        resp = self.auditory_handler.enable(msg.data)
        print(resp)        
   
if __name__ == '__main__':
    LilyAuditory()
