import os
import rospy

import importlib
from actions.std_attention import std_attention
from actions.std_lay import std_lay
from actions.std_paw import std_paw
from actions.std_sit import std_sit
from actions.std_state import std_state
from actions.std_voice import std_voice
from actions.std_sleep import std_sleep
from actions.std_wakeup import std_wakeup
from actions.std_low_bat import std_low_bat
from actions.std_camera import std_camera

from std_msgs.msg import String, Empty, Int32 # TODO

from display_driver.srv import PlayMedia, PlayMediaRequest, PlayMediaResponse
from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import Image

from ears_driver.srv import EarsSetAngle, EarsSetAngleRequest, EarsSetAngleResponse

from neck_driver.srv import NeckSetAngle, NeckSetAngleRequest, NeckSetAngleResponse

from speakers_driver.srv import GetVolume, GetVolumeRequest, GetVolumeResponse
from speakers_driver.srv import PlayAudio, PlayAudioRequest, PlayAudioResponse
from speakers_driver.srv import SetVolume, SetVolumeRequest, SetVolumeResponse

from sensor_msgs.msg import BatteryState # for sensor_driver

class RoboheadController():

    def _display_driver_touchsreen_callback(self, msg:Pose2D):
        print(f"touchsreen callback! {msg.x}, {msg.y}")

    def __init__(self):
        rospy.init_node("robohead_controller")

        # display_driver connect
        self._display_driver_service_PlayMedia_name = "~display_driver/" + rospy.get_param('~display_driver/service_PlayMedia_name')[1:]
        self._display_driver_topic_PlayMedia_name = "~display_driver/" + rospy.get_param('~display_driver/topic_PlayMedia_name')[1:]
        self._display_driver_topic_touchscreen_name = "~display_driver/" + rospy.get_param('~display_driver/topic_touchscreen_name')[1:]
        rospy.wait_for_service(self._display_driver_service_PlayMedia_name)
        self.display_driver_srv_PlayMedia = rospy.ServiceProxy(self._display_driver_service_PlayMedia_name, PlayMedia)
        self.display_driver_pub_PlayMedia = rospy.Publisher(self._display_driver_topic_PlayMedia_name, Image, queue_size=1)
        self.display_driver_sub_touchscreen = rospy.Subscriber(self._display_driver_topic_touchscreen_name, Pose2D, self._display_driver_touchsreen_callback)
        rospy.loginfo("robohead_controller: display_driver connected")

        # ears_driver connect
        self._ears_driver_service_name = "~ears_driver/" + rospy.get_param('~ears_driver/service_name')[1:]
        rospy.wait_for_service(self._ears_driver_service_name)
        self.ears_driver_srv_EarsSetAngle = rospy.ServiceProxy(self._ears_driver_service_name, EarsSetAngle)
        rospy.loginfo("robohead_controller: ears_driver connected")
        
        print(self._display_driver_service_PlayMedia_name)
        print(self._display_driver_topic_PlayMedia_name)
        print(self._display_driver_topic_touchscreen_name)

        rospy.spin()
        
        # self._script_path = os.path.dirname(os.path.abspath(__file__))
        # print("Script path: "+self._script_path)
        # rospy.init_node("head_controller_node")
        
        # self._useMors = rospy.get_param('~useMors', 0)
        # print(f"Use mors: {self._useMors}")

        # rospy.wait_for_service('displayControllerPlay')
        # rospy.wait_for_service('NeckSetAngle')
        # rospy.wait_for_service('EarsSetAngle')
        # rospy.wait_for_service('playSound')

        # self._service_display_player = rospy.ServiceProxy('displayControllerPlay', displayControllerPlay)
        # self._service_set_neck = rospy.ServiceProxy('NeckSetAngle', NeckSetAngle)
        # self._service_set_ears = rospy.ServiceProxy('EarsSetAngle', EarsSetAngle)
        # self._service_play_sound = rospy.ServiceProxy('playSound', playSound)
        
        # self._service_display_player(self._script_path+"/loadingVideo.mp4")




        # print("Servives Inited!")

        # rospy.Subscriber("/head/cmd_from_voice", String, self.process_cmd)
        # rospy.Subscriber("/head/kws_data", String, self.process_kws)
        # rospy.Subscriber("/head/voice_recognizer/grammar_not_found", Empty, self.select_state)

        # rospy.Subscriber("/head/sound_direction", Int32, self.__sound_direction_callback)
        # rospy.Subscriber("/head/bat", BatteryState, self.__battery_state_callback)
        
        # self._hand = 'left'
        # self.is_action = False
        # self._cmd = None
        # self.__sound_direction = 270
        # self._battery_voltage = 4.2
        # self._battery_current = 1
        # self._last_paw = 'left'
        # time.sleep(15)
        # self.run()

    def __battery_state_callback(self, msg:BatteryState):
        self._battery_voltage = msg.voltage
        self._battery_current = msg.current


    def __sound_direction_callback(self, msg:Int32):
        self.__sound_direction = msg.data

    def run(self):
        importlib.reload(std_state)
        self.action_std_state = std_state.STD_STATE(srv_display_player=self._service_display_player,
                                            srv_set_neck=self._service_set_neck,
                                            srv_set_ears=self._service_set_ears,
                                            srv_play_sound=self._service_play_sound,
                                            srv_mors_mode=self._srv_mors_mode,
                                            srv_mors_action=self._srv_mors_action,
                                            srv_mors_cmd_vel=self._srv_mors_cmd_vel,
                                            srv_mors_cmd_pos=self._srv_mors_cmd_pos,
                                            srv_mors_ef_pos=self._srv_mors_ef_pos,
                                            srv_mors_joint_pos=self._srv_mors_joint_pos,
                                            srv_mors_joints_kp=self._srv_mors_joints_kp,
                                            srv_mors_joints_kd=self._srv_mors_joints_kd,
                                            srv_mors_stride_height=self._srv_mors_stride_height,
                                            sound_direction=self.__sound_direction)
        print("Go!")
        resp = self.action_std_state.start_action()

        rospy.spin()

    def select_state(self, msg:Empty):
        if (self._battery_voltage<3.3):
            importlib.reload(std_low_bat)
            self.action_std_low_bat = std_low_bat.STD_LOW_BAT(srv_display_player=self._service_display_player,
                                        srv_set_neck=self._service_set_neck,
                                        srv_set_ears=self._service_set_ears,
                                        srv_play_sound=self._service_play_sound,
                                        srv_mors_mode=self._srv_mors_mode,
                                        srv_mors_action=self._srv_mors_action,
                                        srv_mors_cmd_vel=self._srv_mors_cmd_vel,
                                        srv_mors_cmd_pos=self._srv_mors_cmd_pos,
                                        srv_mors_ef_pos=self._srv_mors_ef_pos,
                                        srv_mors_joint_pos=self._srv_mors_joint_pos,
                                        srv_mors_joints_kp=self._srv_mors_joints_kp,
                                        srv_mors_joints_kd=self._srv_mors_joints_kd,
                                        srv_mors_stride_height=self._srv_mors_stride_height,
                                        sound_direction=self.__sound_direction)
            resp = self.action_std_low_bat.start_action()
            return   
        rospy.sleep(0.1)
        cmd = self._cmd
        if cmd != None:
            if cmd=="голос":
                importlib.reload(std_voice)
                self.action_std_voice = std_voice.STD_VOICE(srv_display_player=self._service_display_player,
                                            srv_set_neck=self._service_set_neck,
                                            srv_set_ears=self._service_set_ears,
                                            srv_play_sound=self._service_play_sound,
                                            srv_mors_mode=self._srv_mors_mode,
                                            srv_mors_action=self._srv_mors_action,
                                            srv_mors_cmd_vel=self._srv_mors_cmd_vel,
                                            srv_mors_cmd_pos=self._srv_mors_cmd_pos,
                                            srv_mors_ef_pos=self._srv_mors_ef_pos,
                                            srv_mors_joint_pos=self._srv_mors_joint_pos,
                                            srv_mors_joints_kp=self._srv_mors_joints_kp,
                                            srv_mors_joints_kd=self._srv_mors_joints_kd,
                                            srv_mors_stride_height=self._srv_mors_stride_height,
                                            sound_direction=self.__sound_direction)
                resp = self.action_std_voice.start_action()

            elif cmd=="сидеть":
                importlib.reload(std_sit)
                self.action_std_sit = std_sit.STD_SIT(srv_display_player=self._service_display_player,
                                            srv_set_neck=self._service_set_neck,
                                            srv_set_ears=self._service_set_ears,
                                            srv_play_sound=self._service_play_sound,
                                            srv_mors_mode=self._srv_mors_mode,
                                            srv_mors_action=self._srv_mors_action,
                                            srv_mors_cmd_vel=self._srv_mors_cmd_vel,
                                            srv_mors_cmd_pos=self._srv_mors_cmd_pos,
                                            srv_mors_ef_pos=self._srv_mors_ef_pos,
                                            srv_mors_joint_pos=self._srv_mors_joint_pos,
                                            srv_mors_joints_kp=self._srv_mors_joints_kp,
                                            srv_mors_joints_kd=self._srv_mors_joints_kd,
                                            srv_mors_stride_height=self._srv_mors_stride_height,
                                            sound_direction=self.__sound_direction)
                resp = self.action_std_sit.start_action()

            elif cmd=="лежать":
                importlib.reload(std_lay)
                self.action_std_lay = std_lay.STD_LAY(srv_display_player=self._service_display_player,
                                            srv_set_neck=self._service_set_neck,
                                            srv_set_ears=self._service_set_ears,
                                            srv_play_sound=self._service_play_sound,
                                            srv_mors_mode=self._srv_mors_mode,
                                            srv_mors_action=self._srv_mors_action,
                                            srv_mors_cmd_vel=self._srv_mors_cmd_vel,
                                            srv_mors_cmd_pos=self._srv_mors_cmd_pos,
                                            srv_mors_ef_pos=self._srv_mors_ef_pos,
                                            srv_mors_joint_pos=self._srv_mors_joint_pos,
                                            srv_mors_joints_kp=self._srv_mors_joints_kp,
                                            srv_mors_joints_kd=self._srv_mors_joints_kd,
                                            srv_mors_stride_height=self._srv_mors_stride_height,
                                            sound_direction=self.__sound_direction)
                resp = self.action_std_lay.start_action()
            elif cmd=="что видишь":
                importlib.reload(std_camera)
                self.action_std_camera = std_camera.STD_CAMERA(srv_display_player=self._service_display_player,
                                            srv_set_neck=self._service_set_neck,
                                            srv_set_ears=self._service_set_ears,
                                            srv_play_sound=self._service_play_sound,
                                            srv_mors_mode=self._srv_mors_mode,
                                            srv_mors_action=self._srv_mors_action,
                                            srv_mors_cmd_vel=self._srv_mors_cmd_vel,
                                            srv_mors_cmd_pos=self._srv_mors_cmd_pos,
                                            srv_mors_ef_pos=self._srv_mors_ef_pos,
                                            srv_mors_joint_pos=self._srv_mors_joint_pos,
                                            srv_mors_joints_kp=self._srv_mors_joints_kp,
                                            srv_mors_joints_kd=self._srv_mors_joints_kd,
                                            srv_mors_stride_height=self._srv_mors_stride_height,
                                            sound_direction=self.__sound_direction)
                resp = self.action_std_camera.start_action()

            elif cmd=="дай левую лапу":
                importlib.reload(std_paw)
                self.action_std_paw = std_paw.STD_PAW(srv_display_player=self._service_display_player,
                                            srv_set_neck=self._service_set_neck,
                                            srv_set_ears=self._service_set_ears,
                                            srv_play_sound=self._service_play_sound,
                                            srv_mors_mode=self._srv_mors_mode,
                                            srv_mors_action=self._srv_mors_action,
                                            srv_mors_cmd_vel=self._srv_mors_cmd_vel,
                                            srv_mors_cmd_pos=self._srv_mors_cmd_pos,
                                            srv_mors_ef_pos=self._srv_mors_ef_pos,
                                            srv_mors_joint_pos=self._srv_mors_joint_pos,
                                            srv_mors_joints_kp=self._srv_mors_joints_kp,
                                            srv_mors_joints_kd=self._srv_mors_joints_kd,
                                            srv_mors_stride_height=self._srv_mors_stride_height,
                                            sound_direction=self.__sound_direction)
                resp = self.action_std_paw.start_action("left")
                self._last_paw = 'left'

            elif cmd=="дай правую лапу":
                importlib.reload(std_paw)
                self.action_std_paw = std_paw.STD_PAW(srv_display_player=self._service_display_player,
                                            srv_set_neck=self._service_set_neck,
                                            srv_set_ears=self._service_set_ears,
                                            srv_play_sound=self._service_play_sound,
                                            srv_mors_mode=self._srv_mors_mode,
                                            srv_mors_action=self._srv_mors_action,
                                            srv_mors_cmd_vel=self._srv_mors_cmd_vel,
                                            srv_mors_cmd_pos=self._srv_mors_cmd_pos,
                                            srv_mors_ef_pos=self._srv_mors_ef_pos,
                                            srv_mors_joint_pos=self._srv_mors_joint_pos,
                                            srv_mors_joints_kp=self._srv_mors_joints_kp,
                                            srv_mors_joints_kd=self._srv_mors_joints_kd,
                                            srv_mors_stride_height=self._srv_mors_stride_height,
                                            sound_direction=self.__sound_direction)
                resp = self.action_std_paw.start_action("right")
                self._last_paw = 'right'

            elif cmd=="дай другую лапу":
                importlib.reload(std_paw)
                self.action_std_paw = std_paw.STD_PAW(srv_display_player=self._service_display_player,
                                            srv_set_neck=self._service_set_neck,
                                            srv_set_ears=self._service_set_ears,
                                            srv_play_sound=self._service_play_sound,
                                            srv_mors_mode=self._srv_mors_mode,
                                            srv_mors_action=self._srv_mors_action,
                                            srv_mors_cmd_vel=self._srv_mors_cmd_vel,
                                            srv_mors_cmd_pos=self._srv_mors_cmd_pos,
                                            srv_mors_ef_pos=self._srv_mors_ef_pos,
                                            srv_mors_joint_pos=self._srv_mors_joint_pos,
                                            srv_mors_joints_kp=self._srv_mors_joints_kp,
                                            srv_mors_joints_kd=self._srv_mors_joints_kd,
                                            srv_mors_stride_height=self._srv_mors_stride_height,
                                            sound_direction=self.__sound_direction)
                if self._last_paw == 'left':
                    resp = self.action_std_paw.start_action("right")
                    self._last_paw = 'right'
                elif self._last_paw == 'right':
                    resp = self.action_std_paw.start_action("left")
                    self._last_paw = 'left'
            elif cmd=="отключись":
                importlib.reload(std_sleep)
                self.action_std_sleep = std_sleep.STD_SLEEP(srv_display_player=self._service_display_player,
                                            srv_set_neck=self._service_set_neck,
                                            srv_set_ears=self._service_set_ears,
                                            srv_play_sound=self._service_play_sound,
                                            srv_mors_mode=self._srv_mors_mode,
                                            srv_mors_action=self._srv_mors_action,
                                            srv_mors_cmd_vel=self._srv_mors_cmd_vel,
                                            srv_mors_cmd_pos=self._srv_mors_cmd_pos,
                                            srv_mors_ef_pos=self._srv_mors_ef_pos,
                                            srv_mors_joint_pos=self._srv_mors_joint_pos,
                                            srv_mors_joints_kp=self._srv_mors_joints_kp,
                                            srv_mors_joints_kd=self._srv_mors_joints_kd,
                                            srv_mors_stride_height=self._srv_mors_stride_height,
                                            sound_direction=self.__sound_direction)
                resp = self.action_std_sleep.start_action()
            elif cmd=="вставай":
                importlib.reload(std_wakeup)
                self.action_std_wakeup = std_wakeup.STD_WAKEUP(srv_display_player=self._service_display_player,
                                            srv_set_neck=self._service_set_neck,
                                            srv_set_ears=self._service_set_ears,
                                            srv_play_sound=self._service_play_sound,
                                            srv_mors_mode=self._srv_mors_mode,
                                            srv_mors_action=self._srv_mors_action,
                                            srv_mors_cmd_vel=self._srv_mors_cmd_vel,
                                            srv_mors_cmd_pos=self._srv_mors_cmd_pos,
                                            srv_mors_ef_pos=self._srv_mors_ef_pos,
                                            srv_mors_joint_pos=self._srv_mors_joint_pos,
                                            srv_mors_joints_kp=self._srv_mors_joints_kp,
                                            srv_mors_joints_kd=self._srv_mors_joints_kd,
                                            srv_mors_stride_height=self._srv_mors_stride_height,
                                            sound_direction=self.__sound_direction)
                resp = self.action_std_wakeup.start_action()

        importlib.reload(std_state)
        self.action_std_state = std_state.STD_STATE(srv_display_player=self._service_display_player,
                                            srv_set_neck=self._service_set_neck,
                                            srv_set_ears=self._service_set_ears,
                                            srv_play_sound=self._service_play_sound,
                                            srv_mors_mode=self._srv_mors_mode,
                                            srv_mors_action=self._srv_mors_action,
                                            srv_mors_cmd_vel=self._srv_mors_cmd_vel,
                                            srv_mors_cmd_pos=self._srv_mors_cmd_pos,
                                            srv_mors_ef_pos=self._srv_mors_ef_pos,
                                            srv_mors_joint_pos=self._srv_mors_joint_pos,
                                            srv_mors_joints_kp=self._srv_mors_joints_kp,
                                            srv_mors_joints_kd=self._srv_mors_joints_kd,
                                            srv_mors_stride_height=self._srv_mors_stride_height,
                                            sound_direction=self.__sound_direction)
        resp = self.action_std_state.start_action()

        # print(f"Resp: {resp}")
        self._cmd = None


    def process_cmd(self, msg:String):
        self._cmd = msg.data

        print(f"Cmd: {msg.data}")
        

    
    def process_kws(self, msg:String):
        print(f"Kws: {msg.data}")
        # self._service_set_neck(15,0)
        # self._service_set_ears(10,10)
        # resp = self._service_display_player("wait_cmd.png")

        importlib.reload(std_attention)
        self.action_std_attention = std_attention.STD_ATTENTION(srv_display_player=self._service_display_player,
                                            srv_set_neck=self._service_set_neck,
                                            srv_set_ears=self._service_set_ears,
                                            srv_play_sound=self._service_play_sound,
                                            srv_mors_mode=self._srv_mors_mode,
                                            srv_mors_action=self._srv_mors_action,
                                            srv_mors_cmd_vel=self._srv_mors_cmd_vel,
                                            srv_mors_cmd_pos=self._srv_mors_cmd_pos,
                                            srv_mors_ef_pos=self._srv_mors_ef_pos,
                                            srv_mors_joint_pos=self._srv_mors_joint_pos,
                                            srv_mors_joints_kp=self._srv_mors_joints_kp,
                                            srv_mors_joints_kd=self._srv_mors_joints_kd,
                                            srv_mors_stride_height=self._srv_mors_stride_height,
                                            sound_direction=self.__sound_direction)
        
        resp = self.action_std_attention.start_action()
        print(f"Resp: {resp}")

if __name__ == "__main__":
	RoboheadController()