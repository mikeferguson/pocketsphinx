#!/usr/bin/python

from time import sleep

import pyaudio

import rospy

from std_msgs.msg import String


class AudioMessage(object):
    """Class to publish audio to topic"""

    def __init__(self):

        # Initializing publisher with buffer size of 10 messages
        self.pub_ = rospy.Publisher("sphinx_audio", String, queue_size=10)

        # initialize node
        rospy.init_node("audio_control")
        # Call custom function on node shutdown
        rospy.on_shutdown(self.shutdown)
        # self.i = 1

        # All set. Publish to topic
        self.transfer_audio_msg()

    def transfer_audio_msg(self):
        """Function to publish input audio to topic"""

        rospy.loginfo("audio input node will start after delay of 5 seconds")
        sleep(5)

        # Params
        self._input = "~input"
        _rate_bool = False

        # Checking if audio file given or system microphone is needed
        if rospy.has_param(self._input):
            if rospy.get_param(self._input) != ":default":
                _rate_bool = True
                stream = open(rospy.get_param(self._input), 'rb')
                rate = rospy.Rate(5) # 10hz
            else:
                # Initializing pyaudio for input from system microhpone
                stream = pyaudio.PyAudio().open(format=pyaudio.paInt16, channels=1,
                                                rate=16000, input=True, frames_per_buffer=1024)
                stream.start_stream()
        else:
            rospy.logerr("No input means provided. Please use the launch file instead")


        while not rospy.is_shutdown():
            buf = stream.read(1024)
            if buf:
                # Publish audio to topic
                self.pub_.publish(buf)
                # rospy.loginfo("COUNT" + str(self.i))
                # self.i += 1


            else:
                rospy.loginfo("Buffer returned null")
                break

    @staticmethod
    def shutdown():
        """This function is executed on node shutdown."""
        # command executed after Ctrl+C is pressed
        rospy.loginfo("Stop ASRControl")
        rospy.sleep(1)


if __name__ == "__main__":
    AudioMessage()
