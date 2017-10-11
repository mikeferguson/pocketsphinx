#!/usr/bin/python

import os
import datetime

import rospy

from std_msgs.msg import String
import pyttsx

def handle_output(data):
    """Map out grammar recognized commands in speech to terminal commands"""
    print (data.data)
    if "go to my workspace" in data.data.lower():
        engine = pyttsx.init()
        engine.setProperty('voice', 'english+f3')
        engine.say("Right away, master!")
        engine.runAndWait()
        os.system("nautilus --browser /home/pankaj/catkin_ws/src/pocketsphinx")
    elif "where is avenger base" in data.data.lower():
        engine = pyttsx.init()
        engine.setProperty('voice', 'english+f3')
        engine.say('I am sorry! That is classified information')
        engine.runAndWait()
    elif "what time is it" in data.data.lower():
        message = 'It is ' + datetime.datetime.now().strftime("%H:%M") + ' hours'
        print (message)
        engine = pyttsx.init()
        engine.setProperty('voice', 'english+f3')
        engine.say(message)
        engine.runAndWait()
    elif "good night jarvis" in data.data.lower():
        message = 'good night, master!'
        engine = pyttsx.init()
        engine.setProperty('voice', 'english+f3')
        engine.say(message)
        engine.runAndWait()

def shutdown():
    """This function is executed on node shutdown."""
    # command executed after Ctrl+C is pressed
    rospy.loginfo("Stop ASRControl")
    rospy.sleep(1)

def init():
    """Initialize node and subscribe to necessary topics"""

    # initialize node
    rospy.init_node("command_control")

    # Call custom function on node shutdown
    rospy.on_shutdown(shutdown)

    rospy.Subscriber("grammar_data", String, handle_output)
    rospy.spin()

if __name__ == "__main__":
    init()
