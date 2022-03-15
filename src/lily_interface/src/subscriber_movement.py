#!/usr/bin/env python3

from handlers.movement import MovementHandler
import rospy
from geometry_msgs.msg import Twist

class LilyMovement(object):

    def __init__(self):
        self.movement_handler = MovementHandler()
        self.next_msg = None

        rospy.init_node('cmd_vel_listener')
        rospy.Subscriber("/cmd_vel", Twist, self.log_msg_request, queue_size=1)

        self.rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.process_action()
    
    def log_msg_request(self, msg): 
        # TODO - Test how well it works for 2 directions (if at all) and adjust.
        # Is this possible with this type of bot?

        # Problem - time taken to process command is too long. Consider.. 
        # Subscriber receives message and adds it to a queue with a timestamp
        # Loop picks up the next message in queue and if not too old, processes it.
        self.next_msg = msg

    def process_action(self):
        if self.next_msg is not None:
            msg = self.next_msg
            self.next_msg = None
            
            if abs(msg.linear.x) > abs(msg.angular.z):
                if msg.linear.x > 0:
                    rospy.loginfo('Movement: forward')
                    result = self.movement_handler.move_forward()
                elif msg.linear.x < 0:
                    rospy.loginfo('Movement: back')
                    result = self.movement_handler.move_back()
            else:
                if msg.angular.z > 0:
                    rospy.loginfo('Movement: left')
                    result = self.movement_handler.move_left()
                elif msg.angular.z < 0:
                    rospy.loginfo('Movement: right')
                    result = self.movement_handler.move_right()
                    
            self.next_msg = None

        self.rate.sleep()
    
if __name__ == '__main__':
    LilyMovement()