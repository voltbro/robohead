import os
import rospy
import importlib

# imports for display_driver
from display_driver.srv import PlayMedia, PlayMediaRequest, PlayMediaResponse
from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import Image

# imports for usb_cam
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
from respeaker_driver.msg import SetColorManualLED
from respeaker_driver.srv import SetBrightnessLED, SetBrightnessLEDRequest, SetBrightnessLEDResponse
from respeaker_driver.srv import SetColorAllLED, SetColorAllLEDRequest, SetColorAllLEDResponse
from respeaker_driver.srv import SetColorPaletteLED, SetColorPaletteLEDRequest, SetColorPaletteLEDResponse
from respeaker_driver.srv import SetModeLED, SetModeLEDRequest, SetModeLEDResponse

# imports for voice_recognizer_pocketsphinx
from voice_recognizer_pocketsphinx.srv import IsWork, IsWorkRequest, IsWorkResponse
from std_msgs.msg import String
from audio_common_msgs.msg import AudioData

# imports for mors
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from sensor_msgs.msg import Imu
from mors.srv import QuadrupedCmd, QuadrupedCmdRequest, QuadrupedCmdResponse
import threading
import numpy as np

class RoboheadController():

    def _execute_action(self, name:str):
        importlib.invalidate_caches()
        try:
            module = self.robohead_controller_actions_match[name]
            action = importlib.import_module(name=module, package=None)
            action = importlib.reload(action)
            action.run(self, name)
        except BaseException as e:
            rospy.logerr(f"Can`t execute command: {name}. Error: {e}")

    def _display_driver_touchsreen_callback(self, msg:Pose2D):
        self.display_driver_touchscreen_xy = (msg.x, msg.y)

    def _sensor_driver_battery_callback(self, msg:BatteryState):
        self.sensor_driver_bat_voltage = msg.voltage
        self.sensor_driver_bat_current = msg.current
        if (self.sensor_driver_bat_voltage < self.robohead_controller_low_voltage_threshold):
            self.robohead_controller_is_allow_work = False
            self._execute_action('low_bat_action')
            rospy.logerr("Low voltage on battery!")
        elif (self.robohead_controller_is_allow_work==False) and (self.sensor_driver_bat_voltage >= self.robohead_controller_low_voltage_threshold+self.robohead_controller_low_voltage_hysteresis):
            self._execute_action('wait_action')
            self.robohead_controller_is_allow_work = True
    
    def _respeaker_driver_audio_main_callback(self, msg:AudioData):
        self.respeaker_driver_msg_audio_main = msg
    def _respeaker_driver_audio_channel_0_callback(self, msg:AudioData):
        self.respeaker_driver_msg_channel_0 = msg
    def _respeaker_driver_audio_channel_1_callback(self, msg:AudioData):
        self.respeaker_driver_msg_channel_1 = msg
    def _respeaker_driver_audio_channel_2_callback(self, msg:AudioData):
        self.respeaker_driver_msg_channel_2 = msg
    def _respeaker_driver_audio_channel_3_callback(self, msg:AudioData):
        self.respeaker_driver_msg_channel_3 = msg
    def _respeaker_driver_audio_channel_4_callback(self, msg:AudioData):
        self.respeaker_driver_msg_channel_4 = msg
    def _respeaker_driver_audio_channel_5_callback(self, msg:AudioData):
        self.respeaker_driver_msg_channel_5 = msg
    def _respeaker_driver_doa_angle_callback(self, msg:Int16):
        self.respeaker_driver_doa_angle = msg.data

    def _voice_recognizer_pocketsphinx_kws_callback(self, msg:String):
        if self.robohead_controller_is_allow_work == False:
            return 
        self.voice_recognizer_pocketsphinx_kws_srv_IsWork(0)
        self._execute_action(msg.data)
        self.voice_recognizer_pocketsphinx_cmds_srv_IsWork(1)
        
    def _voice_recognizer_pocketsphinx_cmds_callback(self, msg:String):
        if self.robohead_controller_is_allow_work == False:
            return 
        self.voice_recognizer_pocketsphinx_cmds_srv_IsWork(0)
        if msg.data != '':
            self._execute_action(msg.data)

        self._execute_action('wait_action')
        self.voice_recognizer_pocketsphinx_kws_srv_IsWork(1)
    
    def _usb_cam_image_raw_callback(self, msg:Image):
        self.usb_cam_image_raw = msg

    def __init__(self):

        self.robohead_controller_low_voltage_threshold = rospy.get_param('~low_voltage_threshold')
        self.robohead_controller_low_voltage_hysteresis = rospy.get_param('~low_voltage_hysteresis')
        self.robohead_controller_is_allow_work = True
        self.robohead_controller_actions_match = rospy.get_param('~robohead_controller_actions_match')

        # display_driver connect
        self.display_driver_touchscreen_xy = (0,0)
        display_driver_service_PlayMedia_name = "~display_driver/" + rospy.get_param('~display_driver/service_PlayMedia_name')[1:]
        display_driver_topic_PlayMedia_name = "~display_driver/" + rospy.get_param('~display_driver/topic_PlayMedia_name')[1:]
        display_driver_topic_touchscreen_name = "~display_driver/" + rospy.get_param('~display_driver/topic_touchscreen_name')[1:]
        rospy.wait_for_service(display_driver_service_PlayMedia_name)
        self.display_driver_srv_PlayMedia = rospy.ServiceProxy(display_driver_service_PlayMedia_name, PlayMedia)
        self.display_driver_pub_PlayMedia = rospy.Publisher(display_driver_topic_PlayMedia_name, Image, queue_size=1)
        self.display_driver_sub_touchscreen = rospy.Subscriber(display_driver_topic_touchscreen_name, Pose2D, self._display_driver_touchsreen_callback)
        msg = PlayMediaRequest()
        msg.is_blocking=0
        msg.is_cycled=1
        msg.path_to_file='/home/pi/robohead_ws/src/robohead/robohead_controller/scripts/loading_splash_2.mp4'
        self.display_driver_srv_PlayMedia(msg)
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
        self.sensor_driver_bat_current = -2.1
        sensor_driver_topic_name = "~sensor_driver/" + rospy.get_param('~sensor_driver/topic_name')[1:]
        rospy.wait_for_message(sensor_driver_topic_name, BatteryState)
        self.sensor_driver_sub_battery = rospy.Subscriber(sensor_driver_topic_name, BatteryState, self._sensor_driver_battery_callback)
        rospy.loginfo("robohead_controller: sensor_driver connected")

        # respeaker_driver connect
        self.respeaker_driver_doa_angle = 0
        self.respeaker_driver_msg_audio_main = AudioData()
        self.respeaker_driver_msg_channel_0 = AudioData()
        self.respeaker_driver_msg_channel_1 = AudioData()
        self.respeaker_driver_msg_channel_2 = AudioData()
        self.respeaker_driver_msg_channel_3 = AudioData()
        self.respeaker_driver_msg_channel_4 = AudioData()
        self.respeaker_driver_msg_channel_5 = AudioData()
        respeaker_driver_topic_audio_main_name = "~respeaker_driver/" + rospy.get_param('~respeaker_driver/ros/topic_audio_main_name')[1:]
        respeaker_driver_topic_audio_channel_0_name = "~respeaker_driver/" + rospy.get_param('~respeaker_driver/ros/topic_audio_channel_0_name')[1:]
        respeaker_driver_topic_audio_channel_1_name = "~respeaker_driver/" + rospy.get_param('~respeaker_driver/ros/topic_audio_channel_1_name')[1:]
        respeaker_driver_topic_audio_channel_2_name = "~respeaker_driver/" + rospy.get_param('~respeaker_driver/ros/topic_audio_channel_2_name')[1:]
        respeaker_driver_topic_audio_channel_3_name = "~respeaker_driver/" + rospy.get_param('~respeaker_driver/ros/topic_audio_channel_3_name')[1:]
        respeaker_driver_topic_audio_channel_4_name = "~respeaker_driver/" + rospy.get_param('~respeaker_driver/ros/topic_audio_channel_4_name')[1:]
        respeaker_driver_topic_audio_channel_5_name = "~respeaker_driver/" + rospy.get_param('~respeaker_driver/ros/topic_audio_channel_5_name')[1:]
        respeaker_driver_topic_doa_angle_name = "~respeaker_driver/" + rospy.get_param('~respeaker_driver/ros/topic_doa_angle_name')[1:]
        respeaker_driver_srv_SetBrightnessLED_name = "~respeaker_driver/" + rospy.get_param('~respeaker_driver/ros/srv_SetBrightnessLED_name')[1:]
        respeaker_driver_srv_SetColorAllLED_name = "~respeaker_driver/" + rospy.get_param('~respeaker_driver/ros/srv_SetColorAllLED_name')[1:]
        respeaker_driver_srv_SetColorPaletteLED_name = "~respeaker_driver/" + rospy.get_param('~respeaker_driver/ros/srv_SetColorPaletteLED_name')[1:]
        respeaker_driver_srv_SetModeLED_name = "~respeaker_driver/" + rospy.get_param('~respeaker_driver/ros/srv_SetModeLED_name')[1:]
        respeaker_driver_topic_SetColorManualLED_name = "~respeaker_driver/" + rospy.get_param('~respeaker_driver/ros/topic_SetColorManualLED_name')[1:]
        self.respeaker_driver_default_led_brightness = rospy.get_param('~respeaker_driver/led/brightness')
        self.respeaker_driver_default_led_mode = rospy.get_param('~respeaker_driver/led/mode')
        self.respeaker_driver_default_led_A_color = rospy.get_param('~respeaker_driver/led/A_color')
        self.respeaker_driver_default_led_B_color = rospy.get_param('~respeaker_driver/led/B_color')
        rospy.wait_for_service(respeaker_driver_srv_SetBrightnessLED_name)
        rospy.wait_for_service(respeaker_driver_srv_SetColorAllLED_name)
        rospy.wait_for_service(respeaker_driver_srv_SetColorPaletteLED_name)
        rospy.wait_for_service(respeaker_driver_srv_SetModeLED_name)
        rospy.wait_for_message(respeaker_driver_topic_audio_main_name, AudioData)
        rospy.wait_for_message(respeaker_driver_topic_audio_channel_0_name, AudioData)
        rospy.wait_for_message(respeaker_driver_topic_audio_channel_1_name, AudioData)
        rospy.wait_for_message(respeaker_driver_topic_audio_channel_2_name, AudioData)
        rospy.wait_for_message(respeaker_driver_topic_audio_channel_3_name, AudioData)
        rospy.wait_for_message(respeaker_driver_topic_audio_channel_4_name, AudioData)
        rospy.wait_for_message(respeaker_driver_topic_audio_channel_5_name, AudioData)
        self.respeaker_driver_sub_audio_main = rospy.Subscriber(respeaker_driver_topic_audio_main_name, AudioData, self._respeaker_driver_audio_main_callback)
        self.respeaker_driver_sub_audio_channel_0 = rospy.Subscriber(respeaker_driver_topic_audio_channel_0_name, AudioData, self._respeaker_driver_audio_channel_0_callback)
        self.respeaker_driver_sub_audio_channel_1 = rospy.Subscriber(respeaker_driver_topic_audio_channel_1_name, AudioData, self._respeaker_driver_audio_channel_1_callback)
        self.respeaker_driver_sub_audio_channel_2 = rospy.Subscriber(respeaker_driver_topic_audio_channel_2_name, AudioData, self._respeaker_driver_audio_channel_2_callback)
        self.respeaker_driver_sub_audio_channel_3 = rospy.Subscriber(respeaker_driver_topic_audio_channel_3_name, AudioData, self._respeaker_driver_audio_channel_3_callback)
        self.respeaker_driver_sub_audio_channel_4 = rospy.Subscriber(respeaker_driver_topic_audio_channel_4_name, AudioData, self._respeaker_driver_audio_channel_4_callback)
        self.respeaker_driver_sub_audio_channel_5 = rospy.Subscriber(respeaker_driver_topic_audio_channel_5_name, AudioData, self._respeaker_driver_audio_channel_5_callback)
        self.respeaker_driver_sub_doa_angle = rospy.Subscriber(respeaker_driver_topic_doa_angle_name, Int16, self._respeaker_driver_doa_angle_callback)
        self.respeaker_driver_pub_SetColorManualLED = rospy.Publisher(respeaker_driver_topic_SetColorManualLED_name, SetColorManualLED, queue_size=1)
        self.respeaker_driver_srv_SetBrightnessLED = rospy.ServiceProxy(respeaker_driver_srv_SetBrightnessLED_name, SetBrightnessLED)
        self.respeaker_driver_srv_SetColorAllLED = rospy.ServiceProxy(respeaker_driver_srv_SetColorAllLED_name, SetColorAllLED)
        self.respeaker_driver_srv_SetColorPaletteLED = rospy.ServiceProxy(respeaker_driver_srv_SetColorPaletteLED_name, SetColorPaletteLED)
        self.respeaker_driver_srv_SetModeLED = rospy.ServiceProxy(respeaker_driver_srv_SetModeLED_name, SetModeLED)
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

        # usb_cam connect
        self.usb_cam_image_raw = Image()
        usb_cam_camera_topic_name = rospy.get_param('~camera_topic_name')
        rospy.wait_for_message(usb_cam_camera_topic_name, Image)
        self.usb_cam_sub_image_raw = rospy.Subscriber(usb_cam_camera_topic_name, Image, self._usb_cam_image_raw_callback)
        rospy.loginfo("robohead_controller: usb_cam connected")

        rospy.logwarn("robohead_controller: all packages connected")
    
    def start_robohead_controller(self):
        self._execute_action("wait_action")
        self.voice_recognizer_pocketsphinx_kws_srv_IsWork(1)
        script_path = os.path.dirname(os.path.abspath(__file__)) + '/'
        msg = PlayAudioRequest()
        msg.is_blocking = 1
        msg.is_cycled = 0
        msg.path_to_file = script_path + "robohead_connected.mp3"
        self.speakers_driver_srv_PlayAudio(msg)

class MorsController(RoboheadController):
    def _mors_topic_radiolink_status_callback(self, msg:Bool):
        self.mors_pub_head_status.publish(not msg.data)

    def _mors_topic_imu_data_callback(self, msg:Imu): 
        w = msg.orientation.w
        x = msg.orientation.x
        y = msg.orientation.y
        z = msg.orientation.z

        ysqr = y * y

        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + ysqr)
        X = np.arctan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)

        t2 = np.clip(t2, a_min=-1.0, a_max=1.0)
        Y = np.arcsin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (ysqr + z * z)
        Z = np.arctan2(t3, t4)

        self.mors_orientation_xyz = (X,Y,Z)

    def mors_action(self, cmd:int) -> int:
        # Выполняет безопасно действия на роботе МОРС
        # возвращает 0 в случае успеха
        # Возвращает -1 при неккоректном номере действия
        # Возвращает -2 при ошибке выполнения действия на роботе
        # 1 - встать
        # 2 - лечь (подготовка к выключению)
        # 3 - дать лапу
        # 4 - кувырок (запрещено!)
        # 5 - Помахать лапой
        # 6 - сидеть
        if (cmd>6) or (cmd<1) or (cmd==4):
            return -1
        
        if (cmd==1) and (self.mors_last_action!=2):
            return 0
        
        if (self.mors_last_action == 2) and (cmd!=1):
            self.mors_srv_robot_action(1)
        result = self.mors_srv_robot_action(cmd)
        self.mors_last_action = cmd
        if result == "get the action": return 0
        else: return -2
    
    def th_mors_mover(self):
        while not rospy.is_shutdown():
            self.mors_pub_head_cmd_vel.publish(self.__mors_cmd_vel)
    def mors_move(self, cmd:Twist) -> int:
        # Выполняет безопасно движение на роботе МОРС
        # возвращает 0 в случае успеха
        # Возвращает -1 при неккоректная скорость
        if abs(cmd.linear.x)>self.mors_constraints_max_linear_vel or abs(cmd.linear.y)>self.mors_constraints_max_linear_vel or cmd.linear.z!=0 or cmd.angular.x!=0 or cmd.angular.y!=0 or abs(cmd.angular.z)>self.mors_constraints_max_angular_vel:
            return -1

        self.mors_srv_robot_mode(0)
        self.__mors_cmd_vel = cmd
        return 0

    def __init__(self):
        super().__init__()
        self.mors_last_action = 2
        self.__mors_cmd_vel = Twist()
        self.mors_orientation_xyz = (0,0,0)

        # Mors connect
        mors_srv_robot_action_name = rospy.get_param('~mors_srv_robot_action_name')
        mors_srv_robot_mode_name = rospy.get_param('~mors_srv_robot_mode_name')
        mors_topic_head_cmd_vel_name = rospy.get_param('~mors_topic_head_cmd_vel_name')
        mors_topic_head_status_name = rospy.get_param('~mors_topic_head_status_name')
        mors_topic_radiolink_status_name = rospy.get_param('~mors_topic_radiolink_status_name')
        mors_topic_imu_data_name = rospy.get_param('~mors_topic_imu_data_name')
        self.mors_constraints_max_linear_vel = rospy.get_param('~mors_constraints/max_linear_vel')
        self.mors_constraints_max_angular_vel = rospy.get_param('~mors_constraints/max_angular_vel')

        rospy.loginfo("mors_controller: wait for topic: "+mors_topic_radiolink_status_name)
        rospy.wait_for_message(mors_topic_radiolink_status_name, Bool)

        rospy.loginfo("mors_controller: wait for topic: "+mors_topic_imu_data_name)
        rospy.wait_for_message(mors_topic_imu_data_name, Imu)

        rospy.loginfo("mors_controller: wait for service: "+mors_srv_robot_action_name)
        rospy.wait_for_service(mors_srv_robot_action_name)

        rospy.loginfo("mors_controller: wait for service: "+mors_srv_robot_mode_name)
        rospy.wait_for_service(mors_srv_robot_mode_name)

        self.mors_pub_head_status = rospy.Publisher(mors_topic_head_status_name, Bool, queue_size=1)
        self.mors_pub_head_cmd_vel = rospy.Publisher(mors_topic_head_cmd_vel_name, Twist, queue_size=1)
        self.mors_srv_robot_action = rospy.ServiceProxy(mors_srv_robot_action_name, QuadrupedCmd)
        self.mors_srv_robot_mode = rospy.ServiceProxy(mors_srv_robot_mode_name, QuadrupedCmd)
        
        self.mors_sub_radiolink_status = rospy.Subscriber(mors_topic_radiolink_status_name, Bool, self._mors_topic_radiolink_status_callback)
        self.mors_sub_imu_data = rospy.Subscriber(mors_topic_imu_data_name, Imu, self._mors_topic_imu_data_callback)
        th = threading.Thread(target=self.th_mors_mover, args=())
        th.start()
        rospy.logwarn("mors_controller: mors connected")
    
    def start_mors_controller(self):
        script_path = os.path.dirname(os.path.abspath(__file__)) + '/'
        msg = PlayAudioRequest()
        msg.is_blocking = 1
        msg.is_cycled = 0
        msg.path_to_file = script_path + "mors_connected.mp3"
        self.speakers_driver_srv_PlayAudio(msg)

if __name__ == "__main__":
    rospy.init_node("mors_controller")
    obj = MorsController()
    obj.start_robohead_controller()
    obj.start_mors_controller()
    rospy.spin()