import rospy
from std_msgs.msg import String
from audio_common_msgs.msg import AudioData

from pocketsphinx.pocketsphinx import *
from sphinxbase.sphinxbase import *
from voice_recognizer_pocketsphinx.srv import IsWork, IsWorkRequest, IsWorkResponse

class KeyWordsRecognizer(object):
    def __init__(self):		
        hmm = rospy.get_param('~hmm')
        dict = rospy.get_param('~dict')
        kws = rospy.get_param('~kws')
        topic_audio_name = rospy.get_param('~topic_audio_name', '/respeaker_driver/audio/main')
        srv_IsWork_name = rospy.get_param('~srv_IsWork_name', '~IsWork')
        default_is_working = rospy.get_param('~default_IsWork', 1)
        self._buffer_size = rospy.get_param('~buffer_size', 5)
        topic_kws_name = rospy.get_param('~topic_kws_name', '~kws')
        logs_output = rospy.get_param('~logs_output', '/dev/null')

        rospy.loginfo(f"wait for topic: {topic_audio_name}")
        rospy.wait_for_message(topic_audio_name, AudioData)
        rospy.loginfo(f"OK: topic exists {topic_audio_name}")
        
        self._buffer_index = 0
        self._buffer = bytearray()
        self._is_working = default_is_working
        
        config = Decoder.default_config()
        config.set_string('-hmm', hmm)
        config.set_string('-dict', dict)
        config.set_string('-kws', kws)
        config.set_string('-logfn', logs_output)

        self._decoder = Decoder(config)
        self._decoder.start_utt()

        self._pub_kws = rospy.Publisher(topic_kws_name, String, queue_size=10)
        rospy.Subscriber(topic_audio_name, AudioData, self.audio_callback)
        rospy.Service(srv_IsWork_name, IsWork, self._requester_IsWork)
        rospy.loginfo("kws_recognizer INITED")
    def _requester_IsWork(self, request:IsWorkRequest):
        response = IsWorkResponse()
        response.Data = self._is_working
        if request.SetStatus == 0:
            self._is_working = False
        elif request.SetStatus == 1:
            self._is_working = True
        return response
    
    def audio_callback(self, msg:AudioData):
        self._buffer += msg.data
        self._buffer_index += 1
        
        if self._buffer_index == self._buffer_size:
            self.process_audio(self._buffer)
            self._buffer = bytearray()
            self._buffer_index = 0
    
        
    def process_audio(self, audio_buffer):
        if self._is_working:
            self._decoder.process_raw(audio_buffer, False, False)
            if self._decoder.hyp() is not None:
                for seg in self._decoder.seg():
                    rospy.loginfo("Detected key word: %s ", seg.word)
                    self._decoder.end_utt()
                    msg = String()
                    msg.data = seg.word
                    self._pub_kws.publish(msg)
                    self._decoder.start_utt()

if __name__ == "__main__":
    rospy.init_node('kws_control')
    KeyWordsRecognizer()
    rospy.spin()