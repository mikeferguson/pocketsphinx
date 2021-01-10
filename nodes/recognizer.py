#!/usr/bin/env python3

"""
recognizer.py is a wrapper for pocketsphinx.
  parameters:
    ~lm - filename of language model
    ~dict - filename of dictionary
    ~hmm - directory name of acoustic model.
                By default it would be "/usr/share/pocketsphinx/model/hmm/en_US/hub4wsj_sc_8k"
                You can download custom acoustic models from CMU Sphix web site: https://sourceforge.net/projects/cmusphinx/files/Acoustic%20and%20Language%20Models/
    ~mic_name - set the pulsesrc device name for the microphone input.
                e.g. a Logitech G35 Headset has the following device name: alsa_input.usb-Logitech_Logitech_G35_Headset-00-Headset_1.analog-mono
                To list audio device info on your machine, in a terminal type: pacmd list-sources
  publications:
    ~output (std_msgs/String) - text output
  services:
    ~start (std_srvs/Empty) - start speech recognition
    ~stop (std_srvs/Empty) - stop speech recognition
"""

import rospy

from gi import pygtkcompat
import gi
gi.require_version('Gst', '1.0')

from gi.repository import GObject, Gst
Gst.init(None)
gst = Gst

pygtkcompat.enable()
pygtkcompat.enable_gtk(version='3.0')

import gtk

from std_msgs.msg import String
from std_srvs.srv import Empty, EmptyResponse

import os
try:
    import commands
except ImportError:
    import subprocess as commands

class recognizer(object):
    """ GStreamer based speech recognizer. """

    def __init__(self):
        # Start node
        rospy.init_node("recognizer")

        self._device_name_param = "~mic_name"  # Find the name of your microphone by typing pacmd list-sources in the terminal
        self._lm_param = "~lm"
        self._dic_param = "~dict"
        self._hmm_param = "~hmm"

        # Configure mics with gstreamer launch config
        if rospy.has_param(self._device_name_param):
            self.device_name = rospy.get_param(self._device_name_param)
            self.device_index = self.pulse_index_from_name(self.device_name)
            self.launch_config = "pulsesrc device=" + str(self.device_index)
            rospy.loginfo("Using: pulsesrc device=%s name=%s", self.device_index, self.device_name)
        elif rospy.has_param('~source'):
            # common sources: 'alsasrc'
            self.launch_config = rospy.get_param('~source')
        else:
            self.launch_config = 'gconfaudiosrc'

        rospy.loginfo("Launch config: %s", self.launch_config)

        self.launch_config += " ! audioconvert ! audioresample " \
                            + '! pocketsphinx name=asr ! fakesink'

        # Configure ROS settings
        self.started = False
        rospy.on_shutdown(self.shutdown)
        self.pub = rospy.Publisher('~output', String, queue_size=10)
        rospy.Service("~start", Empty, self.start)
        rospy.Service("~stop", Empty, self.stop)

        if rospy.has_param(self._lm_param) and rospy.has_param(self._dic_param):
            self.start_recognizer()
        else:
            rospy.logwarn("lm and dic parameters need to be set to start recognizer.")

    def start_recognizer(self):
        rospy.loginfo("Starting recognizer... ")

        self.pipeline = gst.parse_launch(self.launch_config)
        self.asr = self.pipeline.get_by_name('asr')
        self.asr.set_property('dsratio', 1)

        # Configure language model
        if rospy.has_param(self._lm_param):
            lm = rospy.get_param(self._lm_param)
        else:
            rospy.logerr('Recognizer not started. Please specify a language model file.')
            return

        if rospy.has_param(self._dic_param):
            dic = rospy.get_param(self._dic_param)
        else:
            rospy.logerr('Recognizer not started. Please specify a dictionary.')
            return

        if rospy.has_param(self._hmm_param):
            hmm = rospy.get_param(self._hmm_param)


        self.asr.set_property('lm', lm)
        self.asr.set_property('dict', dic)
        if rospy.has_param(self._hmm_param):
            self.asr.set_property('hmm', hmm)

        self.bus = self.pipeline.get_bus()
        self.bus.add_signal_watch()
        self.bus_id = self.bus.connect('message::element', self.element_message)
        self.pipeline.set_state(gst.State.PLAYING)
        self.started = True

    def pulse_index_from_name(self, name):
        output = commands.getstatusoutput("pacmd list-sources | grep -B 1 'name: <" + name + ">' | grep -o -P '(?<=index: )[0-9]*'")

        if len(output) == 2:
            return output[1]
        else:
            raise Exception("Error. pulse index doesn't exist for name: " + name)

    def stop_recognizer(self):
        if self.started:
            self.pipeline.set_state(gst.State.NULL)
            self.pipeline.remove(self.asr)
            self.bus.disconnect(self.bus_id)
            self.started = False

    def shutdown(self):
        """ Delete any remaining parameters so they don't affect next launch """
        for param in [self._device_name_param, self._lm_param, self._dic_param, self._hmm_param]:
            if rospy.has_param(param):
                rospy.delete_param(param)

        """ Shutdown the GTK thread. """
        gtk.main_quit()

    def start(self, req):
        self.start_recognizer()
        rospy.loginfo("recognizer started")
        return EmptyResponse()

    def stop(self, req):
        self.stop_recognizer()
        rospy.loginfo("recognizer stopped")
        return EmptyResponse()

    def element_message(self, bus, msg):
        if msg.get_structure().get_value('final'):
            self.final_result(msg.get_structure().get_value('hypothesis'))
        elif msg.get_structure().get_value('hypothesis'):
            self.partial_result(msg.get_structure().get_value('hypothesis'))

    def partial_result(self, hyp):
        """ Delete any previous selection, insert text and select it. """
        rospy.logdebug("Partial: " + hyp)

    def final_result(self, hyp):
        if len(hyp) > 0:
            """ Insert the final result. """
            msg = String()
            msg.data = str(hyp.lower())
            rospy.loginfo(msg.data)
            self.pub.publish(msg)

if __name__ == "__main__":
    start = recognizer()
    gtk.main()
