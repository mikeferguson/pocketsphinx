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


Creating a new language model:
------------------------------

Creating a new language is straightforward once you have a corpus of possible
sentences your system should handle. To generate the language model and dictionary:

   1. Create `<filename>.corpus` with one sentence per line.
   2. Upload the corpus file here: <http://www.speech.cs.cmu.edu/tools/lmtool-new.html>
   3. Download the .dic and .lm file, and rename them appropriately (`<filename>.dic`).
   4. Change the lm and dict parameters for the recognizer, for example in the launch file.
