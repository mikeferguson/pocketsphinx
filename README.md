A simple ROS wrapper for using Pocketsphinx (via gstreamer) with ROS. See docs here http://wiki.ros.org/pocketsphinx

If installing from source you will need to install the following:
```
sudo apt-get install gstreamer0.10-pocketsphinx
```


Subscribing to ROS audio messages:
---------------------------------

To subscribe to ROS audio messages, pass the `audio_msg_topic` parameter to the
node (normally, this is `/audio`). The recognizer will subscribe to this topic
and use the AudioData messages as input to pocketsphinx.

Requires the [audio_common][1] package.

[1]: http://wiki.ros.org/audio_common
