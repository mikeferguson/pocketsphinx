import sys
import select
import os
import termios
import contextlib
import time


@contextlib.contextmanager
def raw_mode(_file):
    """
    Function to handle the button press on successful utterance of word by user
    """
    old_attrs = termios.tcgetattr(_file.fileno())
    new_attrs = old_attrs[:]
    new_attrs[3] = new_attrs[3] & ~(termios.ECHO | termios.ICANON)
    try:
        termios.tcsetattr(_file.fileno(), termios.TCSADRAIN, new_attrs)
        yield
    finally:
        termios.tcsetattr(_file.fileno(), termios.TCSADRAIN, old_attrs)

def record(_output_file):
    """
    Records user's speech
    """
    os.system('rec -q -c 1 -r 16000 -b 16 ' + _output_file + ' &')
    print ("STARTED RECORDING. PRESS ENTER TO STOP")
    
    with raw_mode(sys.stdin):
        while True:
            if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                _a = sys.stdin.read(1)
                if _a == '\n':
                    print ("STOPPING RECORDING")
                    time.sleep(2)
                    # stop Recording by killing rec process
                    os.system('pkill rec')
                    break

if __name__ == '__main__':
    OUTPUT_FILENAME = 'demo.wav'
    if len(sys.argv) > 1:
        OUTPUT_FILENAME = sys.argv[1] + '.wav'
    record(OUTPUT_FILENAME)
