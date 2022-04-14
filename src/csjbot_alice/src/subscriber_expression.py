#!/usr/bin/env python3

from handlers.expression import ExpressionHandler
import rospy
from std_msgs.msg import String
from csjbot_alice.msg import ExpressionTimed

class LilyExpression(object):

    def __init__(self):
        self.expression_handler = ExpressionHandler()
        self.next_msg = None

        rospy.init_node('expression_listener')
        rospy.Subscriber("/expression_timed", ExpressionTimed, self.process_timed_expression, queue_size=1)
        rospy.Subscriber("/expression", String, self.process_expression, queue_size=1)

        rospy.spin()
        
    def process_expression(self, msg):
        resp = self.expression_handler.express(msg.data)
        print(resp)

        self.rate.sleep()

    def process_timed_expression(self, msg):
        resp = self.expression_handler.express(msg.expression, msg.time)
        print(resp)
        
        self.rate.sleep()
    
if __name__ == '__main__':
    LilyExpression()