#!/usr/bin/python
import rospy

from geometry_msgs.msg import Twist
from std_msgs.msg import String


class ASRControl(object):
    """Class to handle turtlebot simulation control using voice"""

    def __init__(self):

        # Default values for turtlebot_simulator
        self.speed = 0.2
        # Intializing message type
        self.msg = Twist()

        # initialize node
        rospy.init_node("asr_control")
        rospy.on_shutdown(self.shutdown)

        # Initializing publisher with buffer size of 10 messages
        self.pub_ = rospy.Publisher("mobile_base/commands/velocity", Twist, queue_size=10)

        # Subscribe to kws output
        rospy.Subscriber("kws_data", String, self.parse_asr_result)
        rospy.spin()

    def parse_asr_result(self, detected_words): #pylint: disable=too-many-branches
        """Function to perform action on detected word"""
        if detected_words.data.find("full speed") > -1:
            if self.speed == 0.2:
                self.msg.linear.x = self.msg.linear.x * 2
                self.msg.angular.z = self.msg.angular.z * 2
                self.speed = 0.4
        elif detected_words.data.find("half speed") > -1:
            if self.speed == 0.4:
                self.msg.linear.x = self.msg.linear.x / 2
                self.msg.angular.z = self.msg.angular.z / 2
                self.speed = 0.2
        elif detected_words.data.find("forward") > -1:
            self.msg.linear.x = self.speed
            self.msg.angular.z = 0
        elif detected_words.data.find("left") > -1:
            if self.msg.linear.x != 0:
                if self.msg.angular.z < self.speed:
                    self.msg.angular.z += 0.05
            else:
                self.msg.angular.z = self.speed * 2
        elif detected_words.data.find("right") > -1:
            if self.msg.linear.x != 0:
                if self.msg.angular.z > -self.speed:
                    self.msg.angular.z -= 0.05
            else:
                self.msg.angular.z = -self.speed * 2
        elif detected_words.data.find("back") > -1:
            self.msg.linear.x = -self.speed
            self.msg.angular.z = 0
        elif detected_words.data.find("stop") > -1 or detected_words.data.find("halt") > -1:
            self.msg = Twist()

        # Publish required message
        self.pub_.publish(self.msg)

    def shutdown(self):
        """
        command executed after Ctrl+C is pressed
        """
        rospy.loginfo("Stop ASRControl")
        self.pub_.publish(Twist())
        rospy.sleep(1)


if __name__ == "__main__":
    ASRControl()
