# -*- coding: utf-8 -*-


import rospy

import importlib
import actions.std_attention.std_attention
import actions.std_lay.std_lay
import actions.std_paw.std_paw
import actions.std_sit.std_sit
import actions.std_state.std_state
import actions.std_voice.std_voice
import actions.std_sleep.std_sleep
import actions.std_wakeup.std_wakeup

from mors_driver.srv import TwistDuration, PoseArrayDuration, JointTrajectoryPointDuration
from mors_driver.srv import TwistDurationRequest, PoseArrayDurationRequest, JointTrajectoryPointDurationRequest
from mors_driver.srv import TwistDurationResponse, PoseArrayDurationResponse, JointTrajectoryPointDurationResponse

from mors.srv import QuadrupedCmd, QuadrupedCmdRequest, QuadrupedCmdResponse # Необходим пакет mors из mors_base v2.1
from mors.srv import JointsCmd, JointsCmdRequest, JointsCmdResponse

from std_msgs.msg import String, Empty, Int32
from display_controller.srv import displayControllerPlay
from neck_controller.srv import NeckSetAngle
from ears_controller.srv import EarsSetAngle
from speakers_controller.srv import playSound

import time
import os

class HeadController():

    def __init__(self):
        self._script_path = os.path.dirname(os.path.abspath(__file__))
        print("Script path: "+self._script_path)
        rospy.init_node("head_controller_node")

        rospy.wait_for_service('displayControllerPlay')
        rospy.wait_for_service('NeckSetAngle')
        rospy.wait_for_service('EarsSetAngle')
        rospy.wait_for_service('SetMorsAction')
        print("Servives Inited!")
        
        self._service_display_player = rospy.ServiceProxy('displayControllerPlay', displayControllerPlay)
        self._service_set_neck = rospy.ServiceProxy('NeckSetAngle', NeckSetAngle)
        self._service_set_ears = rospy.ServiceProxy('EarsSetAngle', EarsSetAngle)
        self._service_play_sound = rospy.ServiceProxy('playSound', playSound)

        self._srv_mors_mode = rospy.ServiceProxy('SetMorsMode', QuadrupedCmd)
        self._srv_mors_action = rospy.ServiceProxy('SetMorsAction', QuadrupedCmd)
        self._srv_mors_cmd_vel = rospy.ServiceProxy('SetMorsVel', TwistDuration)
        self._srv_mors_cmd_pos = rospy.ServiceProxy('SetMorsPos', TwistDuration)
        self._srv_mors_ef_pos = rospy.ServiceProxy('SetMorsEfPos', PoseArrayDuration)
        self._srv_mors_joint_pos = rospy.ServiceProxy('SetMorsJointPos', JointTrajectoryPointDuration)
        self._srv_mors_joints_kp = rospy.ServiceProxy('SetMorsJointsKp', JointsCmd)
        self._srv_mors_joints_kd = rospy.ServiceProxy('SetMorsJointsKp', JointsCmd)
        self._srv_mors_stride_height = rospy.ServiceProxy('SetMorsStrideHeight', QuadrupedCmd)


        self._sub_cmd_from_voice = rospy.Subscriber("/head/cmd_from_voice", String, self.process_cmd)
        self._sub_cmd_from_voice = rospy.Subscriber("/head/kws_data", String, self.process_kws)
        self._sub_cmd_from_voice = rospy.Subscriber("/head/voice_recognizer/grammar_not_found", Empty, self.default_state)

        self._sub_cmd_from_voice = rospy.Subscriber("/head/sound_direction", Int32, self.__sound_direction_callback)
        
        self._hand = 'left'
        self.is_action = False
        self._cmd = None
        self.__sound_direction = 270
        
        self.run()

    def __sound_direction_callback(self, msg:Int32):
        self.__sound_direction = msg.data

    def run(self):
        importlib.reload(actions.std_state.std_state)
        self.action_std_state = actions.std_state.std_state.STD_STATE(srv_display_player=self._service_display_player,
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

    def default_state(self, msg:Empty):
        
        rospy.sleep(0.1)
        cmd = self._cmd
        if cmd != None:
            if cmd=="голос":
                importlib.reload(actions.std_voice.std_voice)
                self.action_std_voice = actions.std_voice.std_voice.STD_VOICE(srv_display_player=self._service_display_player,
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
                importlib.reload(actions.std_sit.std_sit)
                self.action_std_sit = actions.std_sit.std_sit.STD_SIT(srv_display_player=self._service_display_player,
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
                importlib.reload(actions.std_lay.std_lay)
                self.action_std_lay = actions.std_lay.std_lay.STD_LAY(srv_display_player=self._service_display_player,
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

            elif cmd=="дай левую лапу":
                importlib.reload(actions.std_paw.std_paw)
                self.action_std_paw = actions.std_paw.std_paw.STD_PAW(srv_display_player=self._service_display_player,
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

            elif cmd=="дай правую лапу":
                importlib.reload(actions.std_paw.std_paw)
                self.action_std_paw = actions.std_paw.std_paw.STD_PAW(srv_display_player=self._service_display_player,
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

            elif cmd=="дай другую лапу":
                importlib.reload(actions.std_paw.std_paw)
                self.action_std_paw = actions.std_paw.std_paw.STD_PAW(srv_display_player=self._service_display_player,
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
                resp = self.action_std_paw.start_action("switch")
            elif cmd=="усни":
                importlib.reload(actions.std_sleep.std_sleep)
                self.action_std_sleep = actions.std_sleep.std_sleep.STD_SLEEP(srv_display_player=self._service_display_player,
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
            elif cmd=="проснись":
                importlib.reload(actions.std_wakeup.std_wakeup)
                self.action_std_wakeup = actions.std_wakeup.std_wakeup.STD_WAKEUP(srv_display_player=self._service_display_player,
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

        importlib.reload(actions.std_state.std_state)
        self.action_std_state = actions.std_state.std_state.STD_STATE(srv_display_player=self._service_display_player,
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

        importlib.reload(actions.std_attention.std_attention)
        self.action_std_attention = actions.std_attention.std_attention.STD_ATTENTION(srv_display_player=self._service_display_player,
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
	HeadController()