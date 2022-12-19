#!/usr/bin/env python3

from xmlrpc.client import Boolean
import rospy
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist
from openhab import openHAB
import random

class OpenHABConnector(object):

    def __init__(self):
        rospy.init_node('openhab_connector')
        rospy.Subscriber("/alice/openhab/state/", Bool, self.switch_state)

        self.pub_speech = rospy.Publisher("/speech/", String, queue_size=10, latch=False)
        
        self.openhab_base_url = rospy.get_param("/alice/openhab/base_url", None)
        self.openhab = openHAB(self.openhab_base_url)
        self.openhab_connection_active = False

        self.sensor_list = rospy.get_param("/alice/openhab/sensors/", None)
        self.initialise_sensor_states()
    
        self.listen()

    def initialise_sensor_states(self):
        if self.openhab_connection_active:
            for sensor in self.sensor_list:
                state = self.retrieve_openhab_item_state(sensor)
                if (state != None):
                    self.update_current_sensor_state(sensor, state)
                    
    def update_current_sensor_state(self, sensor, state):
        rospy.set_param("/alice/openhab/" + sensor + "/current/", state)
    
    def fetch_current_sensor_state(self, sensor):
        state = rospy.get_param("/alice/openhab/" + sensor + "/current/", None)
        return state

    def fetch_sensor_joke(self, sensor):
        param_topic = "/alice/openhab/sensors/" + sensor
        jokes = rospy.get_param(param_topic, None)
        rospy.loginfo(jokes)
        joke = None

        if jokes is not None:
            joke = random.choice(jokes)
            rospy.loginfo(joke)

        return joke 

    def fetch_sensor_speech_state(self, sensor, state):
        param_topic = "/alice/openhab/sensors/" + sensor + "/speech-state-" + state
        speech = rospy.get_param(param_topic, None)
        return speech

    def fetch_sensor_turn_state(self, sensor, state):
        param_topic = "/alice/openhab/sensors/" + sensor + "/turn-state-direction-" + state
        turn = rospy.get_param(param_topic, None)

        param_topic = "/alice/openhab/sensors/" + sensor + "/turn-state-duration-" + state
        time = rospy.get_param(param_topic, 0)

        return turn, time

    def listen(self):
        while not rospy.is_shutdown():
            if self.openhab_connection_active:
                for sensor in self.sensor_list:
                    current_state = self.fetch_current_sensor_state(sensor)
                    new_state = self.retrieve_openhab_item_state(sensor)
                    if (current_state != new_state):
                        self.update_current_sensor_state(sensor, new_state)
                        joke = self.fetch_sensor_joke(sensor)
                        if joke is not None:
                            rospy.loginfo(joke[0]['question'])
                            self.pub_speech.publish(joke[0]['question'])
                            # rospy.sleep(1)
                            rospy.loginfo(joke[0]['answer'])
                            self.pub_speech.publish(joke[0]['answer'])

                        speech = self.fetch_sensor_speech_state(sensor, new_state)
                        if speech is not None:
                            self.pub_speech.publish(speech)

                        turn, time = self.fetch_sensor_turn_state(sensor, new_state)
                        if turn is not None:
                            self.turn(turn, time)
                            
            rospy.sleep(0.5)    

    def turn(self, direction, time):
        msg = Twist()
        if direction == 'LEFT':
            msg.angular.z = -0.50
        
        if direction == "RIGHT":
            msg.angular.z = 0.50 

        # Save current time and set publish rate at 10 Hz
        now = rospy.Time.now()
        rate = rospy.Rate(10)

        while rospy.Time.now() < now + rospy.Duration.from_sec(time):
            self.pub_movement.publish(msg)
            rate.sleep()
           
    def switch_state(self, msg):
        self.openhab_connection_active = msg.data

    def retrieve_openhab_item_state(self, sensor_id):
        item = self.openhab.get_item(sensor_id)
        return item.state
        
if __name__ == '__main__':
    OpenHABConnector()
