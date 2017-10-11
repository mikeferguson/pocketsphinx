#!/usr/bin/env python
"""
Major portion taken from:
https://stackoverflow.com/a/11919074

Purpose of this script is to record 
speech input from user as an audio file
"""
import sys, select, os
import termios
import contextlib

import wave
import pyaudio

@contextlib.contextmanager
def raw_mode(file):
    """
    Handles enter key press detection 
    to start or stop recording
    """
    old_attrs = termios.tcgetattr(file.fileno())
    new_attrs = old_attrs[:]
    new_attrs[3] = new_attrs[3] & ~(termios.ECHO | termios.ICANON)
    try:
        termios.tcsetattr(file.fileno(), termios.TCSADRAIN, new_attrs)
        yield
    finally:
        termios.tcsetattr(file.fileno(), termios.TCSADRAIN, old_attrs)

def record(OUTPUT_FILENAME):
    _format = pyaudio.paInt16
    _channels = 1
    _rate = 44100
    _chunk = 1024

    audio = pyaudio.PyAudio()

    _rate = int(audio.get_device_info_by_index(0)['defaultSampleRate'])

    # start Recording
    stream = audio.open(format=_format, channels=_channels,
                        rate=_rate, input=True,
                        frames_per_buffer=_chunk)
    print "STARTED. PRESS ENTER KEY TO STOP RECORDING"
    frames = []

    with raw_mode(sys.stdin):
        while True:
            if not (sys.stdin in select.select([sys.stdin], [], [], 0)[0]):
                data = stream.read(_chunk)
                frames.append(data)
            else:
                a = sys.stdin.read(1)
                print (a)
                if a == '\n':
                    print ("STOPPING RECORDING")
                    # stop Recording
                    stream.stop_stream()
                    stream.close()
                    audio.terminate()

                    _wave_file = wave.open(OUTPUT_FILENAME, 'wb')
                    _wave_file.setnchannels(_channels)
                    _wave_file.setsampwidth(audio.get_sample_size(_format))
                    _wave_file.setframerate(_rate)
                    _wave_file.writeframes(b''.join(frames))
                    _wave_file.close()
                    break


if __name__ == '__main__':
    OUTPUT_FILENAME = 'demo.wav'
    if len(sys.argv) > 1:
        OUTPUT_FILENAME = sys.argv[1] + '.wav'
    record(OUTPUT_FILENAME)
