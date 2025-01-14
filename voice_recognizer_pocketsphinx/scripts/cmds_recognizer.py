import rospy
from std_msgs.msg import String
from audio_common_msgs.msg import AudioData
from pocketsphinx.pocketsphinx import *
from sphinxbase.sphinxbase import *
from voice_recognizer_pocketsphinx.srv import IsWork, IsWorkRequest, IsWorkResponse

class CommandsRecognizer(object):
    def __init__(self):

        hmm = rospy.get_param('~hmm')
        dict = rospy.get_param('~dict')
        gram_file = rospy.get_param('~gram_file')
        grammar_name = rospy.get_param('~grammar_name', "robohead_cmds")
        rule_name = rospy.get_param('~rule_name', "cmds")
        lw = rospy.get_param('~lw', 7.5)
        
        topic_audio_name = rospy.get_param('~topic_audio_name', '/respeaker_driver/audio/main')
        topic_cmds_name = rospy.get_param('~topic_cmds_name', '~commands')
        srv_IsWork_name = rospy.get_param('~srv_IsWork_name', '~IsWork')
        default_is_working = rospy.get_param('~default_IsWork', 1)
        self._buffer_size = rospy.get_param('~buffer_size', 5)
        logs_output = rospy.get_param('~logs_output', '/dev/null')
        self._timeout = rospy.get_param('~timeout', 3)
        
        rospy.loginfo(f"wait for topic: {topic_audio_name}")
        rospy.wait_for_message(topic_audio_name, AudioData)
        rospy.loginfo(f"OK: topic exists {topic_audio_name}")

        self._buffer_index = 0
        self._buffer = bytearray()
        self._is_working = default_is_working
        self._prev_is_speach = False
        self._speach_start = rospy.get_time()

        config = Decoder.default_config()
        config.set_string('-hmm', hmm)
        config.set_string('-dict', dict)
        config.set_string('-logfn', logs_output)
        
        self._decoder = Decoder(config)
        jsgf = Jsgf(gram_file)
        rule = jsgf.get_rule(grammar_name + '.' + rule_name)
        fsg = jsgf.build_fsg(rule, self._decoder.get_logmath(), lw)
        fsg.writefile(gram_file + '.fsg')
        
        self._decoder.set_fsg(gram_file, fsg)
        self._decoder.set_search(gram_file)
        self._decoder.start_utt()
        
        rospy.Subscriber(topic_audio_name, AudioData, self.audio_callback)
        self._pub_cmds = rospy.Publisher(topic_cmds_name, String, queue_size=10)
        rospy.Service(srv_IsWork_name, IsWork, self._requester_IsWork)
        rospy.loginfo("cmds_recognizer INITED")

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
    # def audio_callback(self, msg:AudioData):
    #     self.process_audio(msg.data)
            
    def process_audio(self, audio_buffer):
        if self._is_working:
            self._decoder.process_raw(audio_buffer, False, False)
            is_speach = self._decoder.get_in_speech()
            # print(self._prev_is_speach, is_speach,rospy.get_time()-self._speach_start)
            if (self._prev_is_speach==False) and (is_speach==True):
                self._speach_start = rospy.get_time()
                self._prev_is_speach = True
            
            if ((self._prev_is_speach==True) and (is_speach==False)) or ((self._prev_is_speach==is_speach==True) and ((rospy.get_time()-self._speach_start)>self._timeout)):
                self._prev_is_speach = False
                self._decoder.end_utt()
                if self._decoder.hyp() is not None:
                    msg = String()
                    msg.data = self._decoder.hyp().hypstr
                    # rospy.loginfo("Detected command: %s ", msg.data)
                    self._pub_cmds.publish(msg)
                else:
                    msg = String()
                    msg.data = ""
                    # rospy.loginfo("Detected command: %s ", msg.data)
                    self._pub_cmds.publish(msg)
                self._decoder.start_utt()

if __name__ == "__main__":
    rospy.init_node("asr_control")
    CommandsRecognizer()
    rospy.spin()
    