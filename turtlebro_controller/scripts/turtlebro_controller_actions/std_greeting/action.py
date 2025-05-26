from turtlebro_controller_actions.main import *

def run(turtlebro_controller:TurtlebroController, cmds:str): # Обязательно наличие этой функции, именно она вызывается при голосовой команде
    script_path = os.path.dirname(os.path.abspath(__file__)) + '/'

    msg = SetColorPaletteLEDRequest()
    msg.colorA = [0,255,0]
    msg.colorB = [0,0,255]
    turtlebro_controller.respeaker_driver_srv_SetColorPaletteLED(msg)

    msg = SetModeLEDRequest()
    msg.mode = 4
    turtlebro_controller.respeaker_driver_srv_SetModeLED(msg)

    msg = PlayMediaRequest()
    msg.is_blocking = 1
    msg.is_cycled = 0
    msg.path_to_file = script_path + 'greeting.png'
    turtlebro_controller.display_driver_srv_PlayMedia(msg)

    msg = NeckSetAngleRequest()
    msg.horizontal_angle = 0
    msg.vertical_angle = 30
    msg.duration = 1
    msg.is_blocking = 0
    turtlebro_controller.neck_driver_srv_NeckSetAngle(msg)

    msg = PlayAudioRequest()
    msg.path_to_file = script_path + 'greeting.mp3'
    msg.is_blocking = 1
    msg.is_cycled = 0
    turtlebro_controller.speakers_driver_srv_PlayAudio(msg)