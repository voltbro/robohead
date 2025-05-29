from mors_controller_actions.main import *

import math

def run(mors_controller:MorsController, cmds:str): # Обязательно наличие этой функции, именно она вызывается при голосовой команде
    script_path = os.path.dirname(os.path.abspath(__file__)) + '/'
    
    msg = PlayMediaRequest()
    msg.is_blocking = 0
    msg.is_cycled = 0
    msg.path_to_file = script_path + 'turn_right.png'
    mors_controller.display_driver_srv_PlayMedia(msg)

    msg = PlayAudioRequest()
    msg.path_to_file = script_path + 'turn_right.mp3'
    msg.is_blocking = 0
    msg.is_cycled = 0
    mors_controller.speakers_driver_srv_PlayAudio(msg)

    msg = NeckSetAngleRequest()
    msg.horizontal_angle = 0
    msg.vertical_angle = 30
    msg.duration = 1
    msg.is_blocking = 0
    mors_controller.neck_driver_srv_NeckSetAngle(msg)

    msg = EarsSetAngleRequest()
    msg.left_ear_angle = 0
    msg.right_ear_angle = -30
    mors_controller.ears_driver_srv_EarsSetAngle(msg)
    mors_controller.mors_action(1)
    delta = 0
    prev_theta = mors_controller.mors_orientation_xyz[2] # 0 - x, 1 - y, 2 - z
    cur_delta = 0

    msg = Twist()
    msg.angular.z = -0.7
    msg.linear.x = 0.05 # Необходимо для механического поворота робота
    mors_controller.mors_move(msg)

    while delta<math.pi/2:
        cur_theta = mors_controller.mors_orientation_xyz[2] # 0 - x, 1 - y, 2 - z
        cur_delta = prev_theta-cur_theta
        if (prev_theta<0) and (cur_theta>0):
            cur_delta += 2*math.pi
        if prev_theta != cur_theta:
            prev_theta = cur_theta
        delta += cur_delta

    msg = Twist()
    mors_controller.mors_move(msg)

            
        
        




    