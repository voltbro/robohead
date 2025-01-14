import os
import rospy
import importlib

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
        # print(f"battery callback! {msg.voltage}V, {msg.current}A")
        pass
    
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
        print(f"DOA: {msg.data}")


    def _voice_recognizer_pocketsphinx_kws_callback(self, msg:String):
        print(f"KWS: {msg.data}")
    def _voice_recognizer_pocketsphinx_cmds_callback(self, msg:String):
        print(f"CMDS: {msg.data}")

    

    def __init__(self):

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
        sensor_driver_topic_name = "~sensor_driver/" + rospy.get_param('~sensor_driver/topic_name')[1:]
        rospy.wait_for_message(sensor_driver_topic_name, BatteryState)
        self.sensor_driver_sub_battery = rospy.Subscriber(sensor_driver_topic_name, BatteryState, self._sensor_driver_battery_callback)
        rospy.loginfo("robohead_controller: sensor_driver connected")

        # respeaker_driver connect (optional)
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
        rospy.loginfo("robohead_controller: voice_recognizer_pocketsphinx connected")

        rospy.logwarn("robohead_controller: all packages connected")


        
        # self._script_path = os.path.dirname(os.path.abspath(__file__))
        # print("Script path: "+self._script_path)



        # self.is_action = False
        # self._cmd = None
        # self.__sound_direction = 270
        # self._battery_voltage = 4.2
        # self._battery_current = 1
        # self._last_paw = 'left'
        # time.sleep(15)
        # self.run()



    # def run(self):
    #     importlib.reload(std_state)
    #     self.action_std_state = std_state.STD_STATE(srv_display_player=self._service_display_player,
    #                                         srv_set_neck=self._service_set_neck,
    #                                         srv_set_ears=self._service_set_ears,
    #                                         srv_play_sound=self._service_play_sound,
    #                                         srv_mors_mode=self._srv_mors_mode,
    #                                         srv_mors_action=self._srv_mors_action,
    #                                         srv_mors_cmd_vel=self._srv_mors_cmd_vel,
    #                                         srv_mors_cmd_pos=self._srv_mors_cmd_pos,
    #                                         srv_mors_ef_pos=self._srv_mors_ef_pos,
    #                                         srv_mors_joint_pos=self._srv_mors_joint_pos,
    #                                         srv_mors_joints_kp=self._srv_mors_joints_kp,
    #                                         srv_mors_joints_kd=self._srv_mors_joints_kd,
    #                                         srv_mors_stride_height=self._srv_mors_stride_height,
    #                                         sound_direction=self.__sound_direction)
    #     print("Go!")
    #     resp = self.action_std_state.start_action()

    #     rospy.spin()

    # def select_state(self, msg:Empty):
    #     if (self._battery_voltage<3.3):
    #         importlib.reload(std_low_bat)
    #         self.action_std_low_bat = std_low_bat.STD_LOW_BAT(srv_display_player=self._service_display_player,
    #                                     srv_set_neck=self._service_set_neck,
    #                                     srv_set_ears=self._service_set_ears,
    #                                     srv_play_sound=self._service_play_sound,
    #                                     srv_mors_mode=self._srv_mors_mode,
    #                                     srv_mors_action=self._srv_mors_action,
    #                                     srv_mors_cmd_vel=self._srv_mors_cmd_vel,
    #                                     srv_mors_cmd_pos=self._srv_mors_cmd_pos,
    #                                     srv_mors_ef_pos=self._srv_mors_ef_pos,
    #                                     srv_mors_joint_pos=self._srv_mors_joint_pos,
    #                                     srv_mors_joints_kp=self._srv_mors_joints_kp,
    #                                     srv_mors_joints_kd=self._srv_mors_joints_kd,
    #                                     srv_mors_stride_height=self._srv_mors_stride_height,
    #                                     sound_direction=self.__sound_direction)
    #         resp = self.action_std_low_bat.start_action()
    #         return   
    #     rospy.sleep(0.1)
    #     cmd = self._cmd
    #     if cmd != None:
    #         if cmd=="голос":
    #             importlib.reload(std_voice)
    #             self.action_std_voice = std_voice.STD_VOICE(srv_display_player=self._service_display_player,
    #                                         srv_set_neck=self._service_set_neck,
    #                                         srv_set_ears=self._service_set_ears,
    #                                         srv_play_sound=self._service_play_sound,
    #                                         srv_mors_mode=self._srv_mors_mode,
    #                                         srv_mors_action=self._srv_mors_action,
    #                                         srv_mors_cmd_vel=self._srv_mors_cmd_vel,
    #                                         srv_mors_cmd_pos=self._srv_mors_cmd_pos,
    #                                         srv_mors_ef_pos=self._srv_mors_ef_pos,
    #                                         srv_mors_joint_pos=self._srv_mors_joint_pos,
    #                                         srv_mors_joints_kp=self._srv_mors_joints_kp,
    #                                         srv_mors_joints_kd=self._srv_mors_joints_kd,
    #                                         srv_mors_stride_height=self._srv_mors_stride_height,
    #                                         sound_direction=self.__sound_direction)
    #             resp = self.action_std_voice.start_action()

    #         elif cmd=="сидеть":
    #             importlib.reload(std_sit)
    #             self.action_std_sit = std_sit.STD_SIT(srv_display_player=self._service_display_player,
    #                                         srv_set_neck=self._service_set_neck,
    #                                         srv_set_ears=self._service_set_ears,
    #                                         srv_play_sound=self._service_play_sound,
    #                                         srv_mors_mode=self._srv_mors_mode,
    #                                         srv_mors_action=self._srv_mors_action,
    #                                         srv_mors_cmd_vel=self._srv_mors_cmd_vel,
    #                                         srv_mors_cmd_pos=self._srv_mors_cmd_pos,
    #                                         srv_mors_ef_pos=self._srv_mors_ef_pos,
    #                                         srv_mors_joint_pos=self._srv_mors_joint_pos,
    #                                         srv_mors_joints_kp=self._srv_mors_joints_kp,
    #                                         srv_mors_joints_kd=self._srv_mors_joints_kd,
    #                                         srv_mors_stride_height=self._srv_mors_stride_height,
    #                                         sound_direction=self.__sound_direction)
    #             resp = self.action_std_sit.start_action()

    #         elif cmd=="лежать":
    #             importlib.reload(std_lay)
    #             self.action_std_lay = std_lay.STD_LAY(srv_display_player=self._service_display_player,
    #                                         srv_set_neck=self._service_set_neck,
    #                                         srv_set_ears=self._service_set_ears,
    #                                         srv_play_sound=self._service_play_sound,
    #                                         srv_mors_mode=self._srv_mors_mode,
    #                                         srv_mors_action=self._srv_mors_action,
    #                                         srv_mors_cmd_vel=self._srv_mors_cmd_vel,
    #                                         srv_mors_cmd_pos=self._srv_mors_cmd_pos,
    #                                         srv_mors_ef_pos=self._srv_mors_ef_pos,
    #                                         srv_mors_joint_pos=self._srv_mors_joint_pos,
    #                                         srv_mors_joints_kp=self._srv_mors_joints_kp,
    #                                         srv_mors_joints_kd=self._srv_mors_joints_kd,
    #                                         srv_mors_stride_height=self._srv_mors_stride_height,
    #                                         sound_direction=self.__sound_direction)
    #             resp = self.action_std_lay.start_action()
    #         elif cmd=="что видишь":
    #             importlib.reload(std_camera)
    #             self.action_std_camera = std_camera.STD_CAMERA(srv_display_player=self._service_display_player,
    #                                         srv_set_neck=self._service_set_neck,
    #                                         srv_set_ears=self._service_set_ears,
    #                                         srv_play_sound=self._service_play_sound,
    #                                         srv_mors_mode=self._srv_mors_mode,
    #                                         srv_mors_action=self._srv_mors_action,
    #                                         srv_mors_cmd_vel=self._srv_mors_cmd_vel,
    #                                         srv_mors_cmd_pos=self._srv_mors_cmd_pos,
    #                                         srv_mors_ef_pos=self._srv_mors_ef_pos,
    #                                         srv_mors_joint_pos=self._srv_mors_joint_pos,
    #                                         srv_mors_joints_kp=self._srv_mors_joints_kp,
    #                                         srv_mors_joints_kd=self._srv_mors_joints_kd,
    #                                         srv_mors_stride_height=self._srv_mors_stride_height,
    #                                         sound_direction=self.__sound_direction)
    #             resp = self.action_std_camera.start_action()

    #         elif cmd=="дай левую лапу":
    #             importlib.reload(std_paw)
    #             self.action_std_paw = std_paw.STD_PAW(srv_display_player=self._service_display_player,
    #                                         srv_set_neck=self._service_set_neck,
    #                                         srv_set_ears=self._service_set_ears,
    #                                         srv_play_sound=self._service_play_sound,
    #                                         srv_mors_mode=self._srv_mors_mode,
    #                                         srv_mors_action=self._srv_mors_action,
    #                                         srv_mors_cmd_vel=self._srv_mors_cmd_vel,
    #                                         srv_mors_cmd_pos=self._srv_mors_cmd_pos,
    #                                         srv_mors_ef_pos=self._srv_mors_ef_pos,
    #                                         srv_mors_joint_pos=self._srv_mors_joint_pos,
    #                                         srv_mors_joints_kp=self._srv_mors_joints_kp,
    #                                         srv_mors_joints_kd=self._srv_mors_joints_kd,
    #                                         srv_mors_stride_height=self._srv_mors_stride_height,
    #                                         sound_direction=self.__sound_direction)
    #             resp = self.action_std_paw.start_action("left")
    #             self._last_paw = 'left'

    #         elif cmd=="дай правую лапу":
    #             importlib.reload(std_paw)
    #             self.action_std_paw = std_paw.STD_PAW(srv_display_player=self._service_display_player,
    #                                         srv_set_neck=self._service_set_neck,
    #                                         srv_set_ears=self._service_set_ears,
    #                                         srv_play_sound=self._service_play_sound,
    #                                         srv_mors_mode=self._srv_mors_mode,
    #                                         srv_mors_action=self._srv_mors_action,
    #                                         srv_mors_cmd_vel=self._srv_mors_cmd_vel,
    #                                         srv_mors_cmd_pos=self._srv_mors_cmd_pos,
    #                                         srv_mors_ef_pos=self._srv_mors_ef_pos,
    #                                         srv_mors_joint_pos=self._srv_mors_joint_pos,
    #                                         srv_mors_joints_kp=self._srv_mors_joints_kp,
    #                                         srv_mors_joints_kd=self._srv_mors_joints_kd,
    #                                         srv_mors_stride_height=self._srv_mors_stride_height,
    #                                         sound_direction=self.__sound_direction)
    #             resp = self.action_std_paw.start_action("right")
    #             self._last_paw = 'right'

    #         elif cmd=="дай другую лапу":
    #             importlib.reload(std_paw)
    #             self.action_std_paw = std_paw.STD_PAW(srv_display_player=self._service_display_player,
    #                                         srv_set_neck=self._service_set_neck,
    #                                         srv_set_ears=self._service_set_ears,
    #                                         srv_play_sound=self._service_play_sound,
    #                                         srv_mors_mode=self._srv_mors_mode,
    #                                         srv_mors_action=self._srv_mors_action,
    #                                         srv_mors_cmd_vel=self._srv_mors_cmd_vel,
    #                                         srv_mors_cmd_pos=self._srv_mors_cmd_pos,
    #                                         srv_mors_ef_pos=self._srv_mors_ef_pos,
    #                                         srv_mors_joint_pos=self._srv_mors_joint_pos,
    #                                         srv_mors_joints_kp=self._srv_mors_joints_kp,
    #                                         srv_mors_joints_kd=self._srv_mors_joints_kd,
    #                                         srv_mors_stride_height=self._srv_mors_stride_height,
    #                                         sound_direction=self.__sound_direction)
    #             if self._last_paw == 'left':
    #                 resp = self.action_std_paw.start_action("right")
    #                 self._last_paw = 'right'
    #             elif self._last_paw == 'right':
    #                 resp = self.action_std_paw.start_action("left")
    #                 self._last_paw = 'left'
    #         elif cmd=="отключись":
    #             importlib.reload(std_sleep)
    #             self.action_std_sleep = std_sleep.STD_SLEEP(srv_display_player=self._service_display_player,
    #                                         srv_set_neck=self._service_set_neck,
    #                                         srv_set_ears=self._service_set_ears,
    #                                         srv_play_sound=self._service_play_sound,
    #                                         srv_mors_mode=self._srv_mors_mode,
    #                                         srv_mors_action=self._srv_mors_action,
    #                                         srv_mors_cmd_vel=self._srv_mors_cmd_vel,
    #                                         srv_mors_cmd_pos=self._srv_mors_cmd_pos,
    #                                         srv_mors_ef_pos=self._srv_mors_ef_pos,
    #                                         srv_mors_joint_pos=self._srv_mors_joint_pos,
    #                                         srv_mors_joints_kp=self._srv_mors_joints_kp,
    #                                         srv_mors_joints_kd=self._srv_mors_joints_kd,
    #                                         srv_mors_stride_height=self._srv_mors_stride_height,
    #                                         sound_direction=self.__sound_direction)
    #             resp = self.action_std_sleep.start_action()
    #         elif cmd=="вставай":
    #             importlib.reload(std_wakeup)
    #             self.action_std_wakeup = std_wakeup.STD_WAKEUP(srv_display_player=self._service_display_player,
    #                                         srv_set_neck=self._service_set_neck,
    #                                         srv_set_ears=self._service_set_ears,
    #                                         srv_play_sound=self._service_play_sound,
    #                                         srv_mors_mode=self._srv_mors_mode,
    #                                         srv_mors_action=self._srv_mors_action,
    #                                         srv_mors_cmd_vel=self._srv_mors_cmd_vel,
    #                                         srv_mors_cmd_pos=self._srv_mors_cmd_pos,
    #                                         srv_mors_ef_pos=self._srv_mors_ef_pos,
    #                                         srv_mors_joint_pos=self._srv_mors_joint_pos,
    #                                         srv_mors_joints_kp=self._srv_mors_joints_kp,
    #                                         srv_mors_joints_kd=self._srv_mors_joints_kd,
    #                                         srv_mors_stride_height=self._srv_mors_stride_height,
    #                                         sound_direction=self.__sound_direction)
    #             resp = self.action_std_wakeup.start_action()

    #     importlib.reload(std_state)
    #     self.action_std_state = std_state.STD_STATE(srv_display_player=self._service_display_player,
    #                                         srv_set_neck=self._service_set_neck,
    #                                         srv_set_ears=self._service_set_ears,
    #                                         srv_play_sound=self._service_play_sound,
    #                                         srv_mors_mode=self._srv_mors_mode,
    #                                         srv_mors_action=self._srv_mors_action,
    #                                         srv_mors_cmd_vel=self._srv_mors_cmd_vel,
    #                                         srv_mors_cmd_pos=self._srv_mors_cmd_pos,
    #                                         srv_mors_ef_pos=self._srv_mors_ef_pos,
    #                                         srv_mors_joint_pos=self._srv_mors_joint_pos,
    #                                         srv_mors_joints_kp=self._srv_mors_joints_kp,
    #                                         srv_mors_joints_kd=self._srv_mors_joints_kd,
    #                                         srv_mors_stride_height=self._srv_mors_stride_height,
    #                                         sound_direction=self.__sound_direction)
    #     resp = self.action_std_state.start_action()

    #     # print(f"Resp: {resp}")
    #     self._cmd = None


    # def process_cmd(self, msg:String):
    #     self._cmd = msg.data

    #     print(f"Cmd: {msg.data}")
        

    
    # def process_kws(self, msg:String):
    #     print(f"Kws: {msg.data}")
    #     # self._service_set_neck(15,0)
    #     # self._service_set_ears(10,10)
    #     # resp = self._service_display_player("wait_cmd.png")

    #     importlib.reload(std_attention)
    #     self.action_std_attention = std_attention.STD_ATTENTION(srv_display_player=self._service_display_player,
    #                                         srv_set_neck=self._service_set_neck,
    #                                         srv_set_ears=self._service_set_ears,
    #                                         srv_play_sound=self._service_play_sound,
    #                                         srv_mors_mode=self._srv_mors_mode,
    #                                         srv_mors_action=self._srv_mors_action,
    #                                         srv_mors_cmd_vel=self._srv_mors_cmd_vel,
    #                                         srv_mors_cmd_pos=self._srv_mors_cmd_pos,
    #                                         srv_mors_ef_pos=self._srv_mors_ef_pos,
    #                                         srv_mors_joint_pos=self._srv_mors_joint_pos,
    #                                         srv_mors_joints_kp=self._srv_mors_joints_kp,
    #                                         srv_mors_joints_kd=self._srv_mors_joints_kd,
    #                                         srv_mors_stride_height=self._srv_mors_stride_height,
    #                                         sound_direction=self.__sound_direction)
        
    #     resp = self.action_std_attention.start_action()
    #     print(f"Resp: {resp}")

if __name__ == "__main__":
    rospy.init_node("robohead_controller")
    RoboheadController()
    rospy.spin()