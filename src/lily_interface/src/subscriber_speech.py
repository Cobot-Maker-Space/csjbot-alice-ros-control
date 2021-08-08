#!/usr/bin/env python3

from handlers.speech import SpeechHandler
import rospy
from std_msgs.msg import String

class LilySpeech(object):

    def __init__(self):
        self.speech_handler = SpeechHandler()
        self.next_msg = None

        rospy.init_node('speech_listener')
        rospy.Subscriber("/speech", String, self.process_speech)

        rospy.spin()
    
    def process_speech(self, msg):
        resp = self.speech_handler.speak(msg)
        print(resp)
        # TODO - handle errors here (response code, etc)
        
if __name__ == '__main__':
    LilySpeech()