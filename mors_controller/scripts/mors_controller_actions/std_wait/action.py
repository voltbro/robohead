from mors_controller_actions.main import *

def run(mors_controller:MorsController, cmds:str): # Обязательно наличие этой функции, именно она вызывается при голосовой команде
    script_path = os.path.dirname(os.path.abspath(__file__)) + '/'

    msg = SetModeLEDRequest()
    msg.mode = mors_controller.respeaker_driver_default_led_mode
    mors_controller.respeaker_driver_srv_SetModeLED(msg)

    msg = SetBrightnessLEDRequest()
    msg.brightness = mors_controller.respeaker_driver_default_led_brightness
    mors_controller.respeaker_driver_srv_SetBrightnessLED(msg)

    msg = SetColorPaletteLEDRequest()
    msg.colorA = mors_controller.respeaker_driver_default_led_A_color
    msg.colorB = mors_controller.respeaker_driver_default_led_B_color
    mors_controller.respeaker_driver_srv_SetColorPaletteLED(msg)

    msg = PlayMediaRequest()
    msg.is_blocking = 0
    msg.is_cycled = 0
    msg.path_to_file = script_path + 'wait.png'
    mors_controller.display_driver_srv_PlayMedia(msg)

    msg = EarsSetAngleRequest()
    msg.left_ear_angle = 0
    msg.right_ear_angle = 0
    mors_controller.ears_driver_srv_EarsSetAngle(msg)

    msg = NeckSetAngleRequest()
    msg.horizontal_angle = 0
    msg.vertical_angle = 0
    msg.duration = 1
    msg.is_blocking = 1
    mors_controller.neck_driver_srv_NeckSetAngle(msg)

    msg = PlayAudioRequest()
    msg.path_to_file = ""
    msg.is_blocking = 0
    msg.is_cycled = 0
    mors_controller.speakers_driver_srv_PlayAudio(msg)

    msg = Twist()
    mors_controller.mors_move(msg)
