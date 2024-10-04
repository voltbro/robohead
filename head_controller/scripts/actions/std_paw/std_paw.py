import os
import time
class STD_PAW():
    def __init__(self, srv_display_player=None, srv_set_neck=None, srv_set_ears=None,
                srv_play_sound=None, srv_mors_mode=None, srv_mors_action=None,
                srv_mors_cmd_vel=None, srv_mors_cmd_pos=None, srv_mors_ef_pos=None,
                srv_mors_joint_pos=None, srv_mors_joints_kp=None, srv_mors_joints_kd=None,
                srv_mors_stride_height=None, sound_direction=270):

        self.srv_display_player = srv_display_player
        self.srv_set_neck = srv_set_neck
        self.srv_set_ears = srv_set_ears
        self.srv_play_sound = srv_play_sound

        self.srv_mors_mode=srv_mors_mode
        self.srv_mors_action=srv_mors_action
        self.srv_mors_cmd_vel=srv_mors_cmd_vel
        self.srv_mors_cmd_pos=srv_mors_cmd_pos
        self.srv_mors_ef_pos=srv_mors_ef_pos
        self.srv_mors_joint_pos=srv_mors_joint_pos
        self.srv_mors_joints_kp=srv_mors_joints_kp
        self.srv_mors_joints_kd=srv_mors_joints_kd
        self.srv_mors_stride_height=srv_mors_stride_height
        self.sound_direction=sound_direction

        self._script_path = os.path.dirname(os.path.abspath(__file__))
        self.last_paw = "left"

    def start_action(self, paw:str)->int:

        # Добавить остановку текущего аудио, и его запоминание

        if self.srv_set_neck!=None:
            neck_angle_v = 15 # Change it
            neck_angle_h = 0 # Change it
            duration=1 # Change it
            self.srv_set_neck(neck_angle_v, neck_angle_h, duration)

        if self.srv_set_ears:
            ear_angle_l = 10 # Change it
            ear_angle_r = 10 # Change it
            self.srv_set_ears(ear_angle_l, ear_angle_r)

        if paw=="left":
            self.last_paw = "left"
            path = "/left_paw.png" # Change it
            self.srv_display_player(self._script_path+path)

            path = "/left_paw.mp3" # Change it
            if self.srv_mors_action!=None:
                isBlocking = False # Change it
            else:
                isBlocking = True
            self.srv_play_sound(self._script_path+path, isBlocking)
        elif paw=="right":
            self.last_paw = "right"
            path = "/right_paw.png" # Change it
            self.srv_display_player(self._script_path+path)

            path = "/right_paw.mp3" # Change it
            if self.srv_mors_action!=None:
                isBlocking = False # Change it
            else:
                isBlocking = True
            self.srv_play_sound(self._script_path+path, isBlocking)
        elif paw=="switch":
            if self.last_paw=="right":
                self.last_paw = "left"
                path = "/left_paw.png" # Change it
                self.srv_display_player(self._script_path+path)

                path = "/left_paw.mp3" # Change it
                isBlocking = False # Change it
                self.srv_play_sound(self._script_path+path, isBlocking)
            elif self.last_paw=="left":
                self.last_paw = "right"
                path = "/right_paw.png" # Change it
                self.srv_display_player(self._script_path+path)

                path = "/right_paw.mp3" # Change it
                if self.srv_mors_action!=None:
                    isBlocking = False # Change it
                else:
                    isBlocking = True
                self.srv_play_sound(self._script_path+path, isBlocking)
        
        if self.srv_mors_action!=None:
            self.srv_mors_action(1) # встаем
            if self.last_paw=="left":
                self.srv_mors_action(7) # даем лапу
            if self.last_paw=="right":
                self.srv_mors_action(3) # даем лапу
        # продолжить воспроизводить аудио, которое запомнили в начале
        
        return 0