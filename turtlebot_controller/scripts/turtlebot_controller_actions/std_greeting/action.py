from turtlebot_controller_actions.main import *

def run(robohead_controller:TurtlebotController): # Обязательно наличие этой функции, именно она вызывается при голосовой команде
    script_path = os.path.dirname(os.path.abspath(__file__)) + '/'

    msg = PlayMediaRequest()
    msg.is_blocking = 0
    msg.is_cycled = 0
    msg.path_to_file = script_path + 'greeting.png'
    robohead_controller.display_driver_srv_PlayMedia(msg)

    msg = PlayAudioRequest()
    msg.path_to_file = script_path + 'greeting.mp3'
    msg.is_blocking = 0
    msg.is_cycled = 0
    robohead_controller.speakers_driver_srv_PlayAudio(msg)

    msg = NeckSetAngleRequest()
    msg.horizontal_angle = 0
    msg.vertical_angle = 30
    msg.duration = 1
    msg.is_blocking = 1
    robohead_controller.neck_driver_srv_NeckSetAngle(msg)
    msg.horizontal_angle = 0
    msg.vertical_angle = 0
    msg.duration = 1
    msg.is_blocking = 1
    robohead_controller.neck_driver_srv_NeckSetAngle(msg)