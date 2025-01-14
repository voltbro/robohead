import os
import rospy
import importlib
# heey
# import robohead_controller_actions
# import robohead_controller_actions.std_greeting.action

# from robohead_controller_actions.std_attention import std_attention
# from robohead_controller_actions.std_lay import std_lay
# from robohead_controller_actions.std_paw import std_paw
# from robohead_controller_actions.std_sit import std_sit
# from actions.std_state import std_state
# from actions.std_voice import std_voice
# from actions.std_sleep import std_sleep
# from actions.std_wakeup import std_wakeup
# from actions.std_low_bat import std_low_bat
# from actions.std_camera import std_camera

# imports for display_driver
from display_driver.srv import PlayMedia, PlayMediaRequest, PlayMediaResponse
from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import Image

# imports for cv_camera
from sensor_msgs.msg import Image

# imports for ears_driver
from ears_driver.srv import EarsSetAngle, EarsSetAngleRequest, EarsSetAngleResponse

# imports for neck_driver
from neck_driver.srv import NeckSetAngle, NeckSetAngleRequest, NeckSetAngleResponse

# imports for speakers_driver
from speakers_driver.srv import GetVolume, GetVolumeRequest, GetVolumeResponse
from speakers_driver.srv import PlayAudio, PlayAudioRequest, PlayAudioResponse
from speakers_driver.srv import SetVolume, SetVolumeRequest, SetVolumeResponse

# imports for sensor_driver
from sensor_msgs.msg import BatteryState

# imports for respeaker_driver
from std_msgs.msg import Int16
from audio_common_msgs.msg import AudioData

# imports for voice_recognizer_pocketsphinx
from voice_recognizer_pocketsphinx.srv import IsWork, IsWorkRequest, IsWorkResponse
from std_msgs.msg import String
from audio_common_msgs.msg import AudioData


class RoboheadController():

    def _display_driver_touchsreen_callback(self, msg:Pose2D):
        print(f"touchsreen callback! xy: {msg.x}, {msg.y}")

    def _sensor_driver_battery_callback(self, msg:BatteryState):
        self.sensor_driver_bat_voltage = msg.voltage
        self.sensor_driver_bat_current = msg.current
        if self.sensor_driver_bat_voltage < self.low_voltage_threshold:
            rospy.logerr("Low voltage on battery!")
    
    def _respeaker_driver_audio_main_callback(self, msg:AudioData):
        pass
    def _respeaker_driver_audio_channel_0_callback(self, msg:AudioData):
        pass
    def _respeaker_driver_audio_channel_1_callback(self, msg:AudioData):
        pass
    def _respeaker_driver_audio_channel_2_callback(self, msg:AudioData):
        pass
    def _respeaker_driver_audio_channel_3_callback(self, msg:AudioData):
        pass
    def _respeaker_driver_audio_channel_4_callback(self, msg:AudioData):
        pass
    def _respeaker_driver_audio_channel_5_callback(self, msg:AudioData):
        pass
    def _respeaker_driver_doa_angle_callback(self, msg:Int16):
        self.respeaker_driver_doa_angle = msg.data
        # print(f"DOA: {msg.data}")

    def _voice_recognizer_pocketsphinx_kws_callback(self, msg:String):
        # print(f"KWS: {msg.data}")
        self.voice_recognizer_pocketsphinx_kws_srv_IsWork(0)

        importlib.invalidate_caches()
        try:
            module = self.robohead_controller_actions_match[msg.data]
            action = importlib.import_module(name=module, package=None)
            action = importlib.reload(action)
            action.run(self)
        except BaseException as e:
            rospy.logerr(f"Can`t execute command: {msg.data}. Error: {e}")

        self.voice_recognizer_pocketsphinx_cmds_srv_IsWork(1)
        
    def _voice_recognizer_pocketsphinx_cmds_callback(self, msg:String):
        self.voice_recognizer_pocketsphinx_cmds_srv_IsWork(0)
        if msg.data != '':
            importlib.invalidate_caches()
            try:
                module = self.robohead_controller_actions_match[msg.data]
                action = importlib.import_module(name=module, package=None)
                action = importlib.reload(action)
                action.run(self)
            except BaseException as e:
                rospy.logerr(f"Can`t execute command: {msg.data}. Error: {e}")

            importlib.invalidate_caches()
            try:
                module = self.robohead_controller_actions_match['wait']
                action = importlib.import_module(name=module, package=None)
                action = importlib.reload(action)
                action.run(self)
            except BaseException as e:
                rospy.logerr(f"Can`t execute command: 'wait'. Error: {e}")
        self.voice_recognizer_pocketsphinx_kws_srv_IsWork(1)
    
    def _cv_camera_image_raw_callback(self, msg:Image):
        self.cv_camera_image_raw = msg.data

    def __init__(self):

        self.low_voltage_threshold = rospy.get_param('~low_voltage_threshold')
        self.robohead_controller_actions_match = rospy.get_param('~robohead_controller_actions_match')

        # display_driver connect
        display_driver_service_PlayMedia_name = "~display_driver/" + rospy.get_param('~display_driver/service_PlayMedia_name')[1:]
        display_driver_topic_PlayMedia_name = "~display_driver/" + rospy.get_param('~display_driver/topic_PlayMedia_name')[1:]
        display_driver_topic_touchscreen_name = "~display_driver/" + rospy.get_param('~display_driver/topic_touchscreen_name')[1:]
        rospy.wait_for_service(display_driver_service_PlayMedia_name)
        self.display_driver_srv_PlayMedia = rospy.ServiceProxy(display_driver_service_PlayMedia_name, PlayMedia)
        self.display_driver_pub_PlayMedia = rospy.Publisher(display_driver_topic_PlayMedia_name, Image, queue_size=1)
        self.display_driver_sub_touchscreen = rospy.Subscriber(display_driver_topic_touchscreen_name, Pose2D, self._display_driver_touchsreen_callback)
        rospy.loginfo("robohead_controller: display_driver connected")

        # ears_driver connect
        ears_driver_service_name = "~ears_driver/" + rospy.get_param('~ears_driver/service_name')[1:]
        rospy.wait_for_service(ears_driver_service_name)
        self.ears_driver_srv_EarsSetAngle = rospy.ServiceProxy(ears_driver_service_name, EarsSetAngle)
        rospy.loginfo("robohead_controller: ears_driver connected")

        # neck_driver connect
        neck_driver_service_name = "~neck_driver/" + rospy.get_param('~neck_driver/service_name')[1:]
        rospy.wait_for_service(neck_driver_service_name)
        self.neck_driver_srv_NeckSetAngle = rospy.ServiceProxy(neck_driver_service_name, NeckSetAngle)
        rospy.loginfo("robohead_controller: neck_driver connected")

        # speakers_driver connect
        speakers_driver_service_PlayAudio_name = "~speakers_driver/" + rospy.get_param('~speakers_driver/service_PlayAudio_name')[1:]
        speakers_driver_service_GetVolume_name = "~speakers_driver/" + rospy.get_param('~speakers_driver/service_GetVolume_name')[1:]
        speakers_driver_service_SetVolume_name = "~speakers_driver/" + rospy.get_param('~speakers_driver/service_SetVolume_name')[1:]
        rospy.wait_for_service(speakers_driver_service_PlayAudio_name)
        rospy.wait_for_service(speakers_driver_service_GetVolume_name)
        rospy.wait_for_service(speakers_driver_service_SetVolume_name)
        self.speakers_driver_srv_PlayAudio = rospy.ServiceProxy(speakers_driver_service_PlayAudio_name, PlayAudio)
        self.speakers_driver_srv_GetVolume = rospy.ServiceProxy(speakers_driver_service_PlayAudio_name, GetVolume)
        self.speakers_driver_srv_SetVolume = rospy.ServiceProxy(speakers_driver_service_PlayAudio_name, SetVolume)
        rospy.loginfo("robohead_controller: speakers_driver connected")

        # sensor_driver connect
        self.sensor_driver_bat_voltage = 4.2
        self.sensor_driver_bat_current = -2
        sensor_driver_topic_name = "~sensor_driver/" + rospy.get_param('~sensor_driver/topic_name')[1:]
        rospy.wait_for_message(sensor_driver_topic_name, BatteryState)
        self.sensor_driver_sub_battery = rospy.Subscriber(sensor_driver_topic_name, BatteryState, self._sensor_driver_battery_callback)
        rospy.loginfo("robohead_controller: sensor_driver connected")

        # respeaker_driver connect
        self.respeaker_driver_doa_angle = 0
        respeaker_driver_topic_audio_main_name = "~respeaker_driver/" + rospy.get_param('~respeaker_driver/ros/topic_audio_main_name')[1:]
        respeaker_driver_topic_audio_channel_0_name = "~respeaker_driver/" + rospy.get_param('~respeaker_driver/ros/topic_audio_channel_0_name')[1:]
        respeaker_driver_topic_audio_channel_1_name = "~respeaker_driver/" + rospy.get_param('~respeaker_driver/ros/topic_audio_channel_1_name')[1:]
        respeaker_driver_topic_audio_channel_2_name = "~respeaker_driver/" + rospy.get_param('~respeaker_driver/ros/topic_audio_channel_2_name')[1:]
        respeaker_driver_topic_audio_channel_3_name = "~respeaker_driver/" + rospy.get_param('~respeaker_driver/ros/topic_audio_channel_3_name')[1:]
        respeaker_driver_topic_audio_channel_4_name = "~respeaker_driver/" + rospy.get_param('~respeaker_driver/ros/topic_audio_channel_4_name')[1:]
        respeaker_driver_topic_audio_channel_5_name = "~respeaker_driver/" + rospy.get_param('~respeaker_driver/ros/topic_audio_channel_5_name')[1:]
        respeaker_driver_topic_doa_angle_name = "~respeaker_driver/" + rospy.get_param('~respeaker_driver/ros/topic_doa_angle_name')[1:]
        rospy.wait_for_message(respeaker_driver_topic_audio_main_name, AudioData)
        rospy.wait_for_message(respeaker_driver_topic_audio_channel_0_name, AudioData)
        rospy.wait_for_message(respeaker_driver_topic_audio_channel_1_name, AudioData)
        rospy.wait_for_message(respeaker_driver_topic_audio_channel_2_name, AudioData)
        rospy.wait_for_message(respeaker_driver_topic_audio_channel_3_name, AudioData)
        rospy.wait_for_message(respeaker_driver_topic_audio_channel_4_name, AudioData)
        rospy.wait_for_message(respeaker_driver_topic_audio_channel_5_name, AudioData)
        self.respeaker_driver_sub_audio_main = rospy.Subscriber(respeaker_driver_topic_audio_main_name, AudioData, self._respeaker_driver_audio_main_callback)
        self.respeaker_driver_sub_audio_channel_0 = rospy.Subscriber(respeaker_driver_topic_audio_channel_0_name, AudioData, self._respeaker_driver_audio_channel_0_callback)
        self.respeaker_driver_sub_audio_channel_0 = rospy.Subscriber(respeaker_driver_topic_audio_channel_1_name, AudioData, self._respeaker_driver_audio_channel_1_callback)
        self.respeaker_driver_sub_audio_channel_0 = rospy.Subscriber(respeaker_driver_topic_audio_channel_2_name, AudioData, self._respeaker_driver_audio_channel_2_callback)
        self.respeaker_driver_sub_audio_channel_0 = rospy.Subscriber(respeaker_driver_topic_audio_channel_3_name, AudioData, self._respeaker_driver_audio_channel_3_callback)
        self.respeaker_driver_sub_audio_channel_0 = rospy.Subscriber(respeaker_driver_topic_audio_channel_4_name, AudioData, self._respeaker_driver_audio_channel_4_callback)
        self.respeaker_driver_sub_audio_channel_0 = rospy.Subscriber(respeaker_driver_topic_audio_channel_5_name, AudioData, self._respeaker_driver_audio_channel_5_callback)
        self.respeaker_driver_sub_doa_angle = rospy.Subscriber(respeaker_driver_topic_doa_angle_name, Int16, self._respeaker_driver_doa_angle_callback)
        rospy.loginfo("robohead_controller: respeaker_driver connected")

        # voice_recognizer_pocketsphinx connect
        voice_recognizer_pocketsphinx_cmds_recognizer_topic_cmds_name = "~voice_recognizer_pocketsphinx/cmds_recognizer/" + rospy.get_param('~voice_recognizer_pocketsphinx/cmds_recognizer/topic_cmds_name')[1:]
        voice_recognizer_pocketsphinx_cmds_recognizer_srv_IsWork_name = "~voice_recognizer_pocketsphinx/cmds_recognizer/" + rospy.get_param('~voice_recognizer_pocketsphinx/cmds_recognizer/srv_IsWork_name')[1:]
        voice_recognizer_pocketsphinx_kws_recognizer_topic_kws_name = "~voice_recognizer_pocketsphinx/kws_recognizer/" + rospy.get_param('~voice_recognizer_pocketsphinx/kws_recognizer/topic_kws_name')[1:]
        voice_recognizer_pocketsphinx_kws_recognizer_srv_IsWork_name = "~voice_recognizer_pocketsphinx/kws_recognizer/" + rospy.get_param('~voice_recognizer_pocketsphinx/kws_recognizer/srv_IsWork_name')[1:]
        rospy.wait_for_service(voice_recognizer_pocketsphinx_cmds_recognizer_srv_IsWork_name)
        rospy.wait_for_service(voice_recognizer_pocketsphinx_kws_recognizer_srv_IsWork_name)
        self.voice_recognizer_pocketsphinx_sub_kws = rospy.Subscriber(voice_recognizer_pocketsphinx_kws_recognizer_topic_kws_name, String, self._voice_recognizer_pocketsphinx_kws_callback)
        self.voice_recognizer_pocketsphinx_sub_cmds = rospy.Subscriber(voice_recognizer_pocketsphinx_cmds_recognizer_topic_cmds_name, String, self._voice_recognizer_pocketsphinx_cmds_callback)
        self.voice_recognizer_pocketsphinx_kws_srv_IsWork = rospy.ServiceProxy(voice_recognizer_pocketsphinx_kws_recognizer_srv_IsWork_name, IsWork)
        self.voice_recognizer_pocketsphinx_cmds_srv_IsWork = rospy.ServiceProxy(voice_recognizer_pocketsphinx_cmds_recognizer_srv_IsWork_name, IsWork)
        self.voice_recognizer_pocketsphinx_cmds_srv_IsWork(0) # Выключаем распознавание команд
        self.voice_recognizer_pocketsphinx_kws_srv_IsWork(0) # Выключаем распознавание ключевых слов
        rospy.loginfo("robohead_controller: voice_recognizer_pocketsphinx connected")

        # cv_camera connect
        self.cv_camera_image_raw = [0]
        cv_camera_camera_topic_name = rospy.get_param('~/camera_topic_name')
        rospy.wait_for_message(cv_camera_camera_topic_name, Image)
        self.cv_camera_sub_image_raw = rospy.Subscriber(cv_camera_camera_topic_name, Image, self._cv_camera_image_raw_callback)
        rospy.loginfo("robohead_controller: cv_camera connected")

        rospy.logwarn("robohead_controller: all packages connected")

        importlib.invalidate_caches()
        try:
            module = self.robohead_controller_actions_match['wait']
            action = importlib.import_module(name=module, package=None)
            action = importlib.reload(action)
            action.run(self)
        except BaseException as e:
            rospy.logerr(f"Can`t execute command: 'wait'. Error: {e}")

        self.voice_recognizer_pocketsphinx_kws_srv_IsWork(1)

if __name__ == "__main__":
    rospy.init_node("robohead_controller")
    RoboheadController()
    rospy.spin()