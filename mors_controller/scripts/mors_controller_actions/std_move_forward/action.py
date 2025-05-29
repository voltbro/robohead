from mors_controller_actions.main import *

import math

def run(mors_controller:MorsController, cmds:str): # Обязательно наличие этой функции, именно она вызывается при голосовой команде
    script_path = os.path.dirname(os.path.abspath(__file__)) + '/'
    
    msg = PlayMediaRequest()
    msg.is_blocking = 0
    msg.is_cycled = 0
    msg.path_to_file = script_path + 'move_away.png'
    mors_controller.display_driver_srv_PlayMedia(msg)

    msg = PlayAudioRequest()
    msg.path_to_file = script_path + 'move_away_start.mp3'
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
    msg = Twist()
    msg.linear.x = 0.1
    
    start = rospy.get_time()
    print("start stepping")
    mors_controller.mors_move(msg)
    rospy.sleep(5) # Шагаем 5 секунд
    print("end stepping")
    msg = Twist()
    mors_controller.mors_move(msg)

    msg = PlayAudioRequest()
    msg.path_to_file = script_path + 'move_away_finish.mp3'
    msg.is_blocking = 1
    msg.is_cycled = 0
    mors_controller.speakers_driver_srv_PlayAudio(msg)