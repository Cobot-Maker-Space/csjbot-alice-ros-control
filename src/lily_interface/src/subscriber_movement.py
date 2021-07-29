#!/usr/bin/env python3

from handlers.movement import MovementHandler
import rospy
from geometry_msgs.msg import Twist

class LilyMovement(object):

    def __init__(self):
        self.movement_handler = MovementHandler()

        rospy.init_node('cmd_vel_listener')
        rospy.Subscriber("/cmd_vel", Twist, self.action_movement)
        rospy.spin()
    
    def action_movement(self, msg): 
        # TODO - Test how well it works for 2 directions (if at all) and adjust.
        # Is this possible with this type of bot?

        if msg.linear.x > 0:
            rospy.loginfo('Movement: forward')
            self.movement_handler.move_forward()
        
        if msg.linear.x < 0:
            rospy.loginfo('Movement: back')
            self.movement_handler.move_back()
        
        if msg.angular.z > 0:
            rospy.loginfo('Movement: left')
            self.movement_handler.move_left()
        
        if msg.angular.z < 0:
            rospy.loginfo('Movement: right')
            self.movement_handler.move_right()

        pass

if __name__ == '__main__':
    LilyMovement()