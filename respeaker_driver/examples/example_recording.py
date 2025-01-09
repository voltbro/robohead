from audio_common_msgs.msg import AudioData

import rospy
import os
import sys
import pyaudio
from contextlib import contextmanager
import wave

@contextmanager
def ignore_stderr(enable=True):
    if enable:
        devnull = None
        try:
            devnull = os.open(os.devnull, os.O_WRONLY)
            stderr = os.dup(2)
            sys.stderr.flush()
            os.dup2(devnull, 2)
            try:
                yield
            finally:
                os.dup2(stderr, 2)
                os.close(stderr)
        finally:
            if devnull is not None:
                os.close(devnull)
    else:
        yield
    
class Recorder():
    def __init__(self, filename):
        self.filename = filename
        self.CHUNK = 1024
        self.RATE = 16000
        self.RECORD_SECONDS = 5

        with ignore_stderr():
            p = pyaudio.PyAudio()
        self.format = p.get_sample_size(pyaudio.paInt16)
        p.terminate()

        self.frames = []
        self.i = 0
        self.flag = 0

    def callback(self, msg:AudioData):

        print(self.i, (self.RATE / self.CHUNK * self.RECORD_SECONDS))
        self.frames.append(msg.data)
        self.i += 1
        if (self.i >= int(self.RATE / self.CHUNK * self.RECORD_SECONDS)) and self.flag==0:
            self.flag = 1
            wf = wave.open(self.filename, 'wb')
            wf.setnchannels(1)
            wf.setsampwidth(self.format)
            wf.setframerate(self.RATE)
            wf.writeframes(b''.join(self.frames))
            wf.close()
            print("recorded!")
            
            exit(0)


r0 = Recorder("rec0.wav") # channel 0 - обработанный звук самим respeaker
r1 = Recorder("rec1.wav") # channel 1 - raw звук с микрофона 1
r2 = Recorder("rec2.wav") # channel 2 - raw звук с микрофона 2
r3 = Recorder("rec3.wav") # channel 3 - raw звук с микрофона 3
r4 = Recorder("rec4.wav") # channel 4 - raw звук с микрофона 4
r5 = Recorder("rec5.wav") # channel 5 - звук, воспроизводимый на динамиках

r = Recorder("rec.wav") # channel main (0)
rospy.init_node("example_recording")

rospy.Subscriber("/head/audio/channel0", AudioData, r0.callback)
rospy.Subscriber("/head/audio/channel1", AudioData, r1.callback)
rospy.Subscriber("/head/audio/channel2", AudioData, r2.callback)
rospy.Subscriber("/head/audio/channel3", AudioData, r3.callback)
rospy.Subscriber("/head/audio/channel4", AudioData, r4.callback)
rospy.Subscriber("/head/audio/channel5", AudioData, r5.callback)
rospy.Subscriber("/head/audio", AudioData, r.callback)

rospy.spin()

