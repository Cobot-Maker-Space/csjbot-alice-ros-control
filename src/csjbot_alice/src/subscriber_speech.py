#!/usr/bin/env python3

from handlers.speech import SpeechHandler
import rospy
from std_msgs.msg import String
from csjbot_alice.msg import SpeechSettings

class LilySpeech(object):

    def __init__(self):
        self.speech_handler = SpeechHandler()

        rospy.init_node('speech_listener')
        rospy.Subscriber("/speech", String, self.process_speech)
        rospy.Subscriber("/speech_settings", SpeechSettings, self.update_settings)
        rospy.Subscriber("/speech_settings/voice_name", String, self.update_voice)

        rospy.spin()

    def process_speech(self, msg):
        resp = self.speech_handler.speak(msg)
        print(resp)
        # TODO - handle errors here (response code, etc)

    def update_settings(self, msg):
        resp = self.speech_handler.update_settings(voice=msg.voice_name, pitch=msg.pitch, speed=msg.speed, rnd=msg.rnd)
        print(resp)
        # TODO - handle errors here (response code, etc)

    def update_voice(self, msg):
        resp = self.speech_handler.update_settings(voice=msg.data)
        print(resp)
        # TODO - handle errors here (response code, etc)

if __name__ == '__main__':
    LilySpeech()
