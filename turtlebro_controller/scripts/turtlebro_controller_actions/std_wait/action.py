from turtlebro_controller_actions.main import *

def run(turtlebro_controller:TurtlebroController, cmds:str): # Обязательно наличие этой функции, именно она вызывается при голосовой команде
    script_path = os.path.dirname(os.path.abspath(__file__)) + '/'

    msg = SetModeLEDRequest()
    msg.mode = turtlebro_controller.respeaker_driver_default_led_mode
    turtlebro_controller.respeaker_driver_srv_SetModeLED(msg)

    msg = SetBrightnessLEDRequest()
    msg.brightness = turtlebro_controller.respeaker_driver_default_led_brightness
    turtlebro_controller.respeaker_driver_srv_SetBrightnessLED(msg)

    msg = SetColorPaletteLEDRequest()
    msg.colorA = turtlebro_controller.respeaker_driver_default_led_A_color
    msg.colorB = turtlebro_controller.respeaker_driver_default_led_B_color
    turtlebro_controller.respeaker_driver_srv_SetColorPaletteLED(msg)

    msg = PlayMediaRequest()
    msg.is_blocking = 0
    msg.is_cycled = 0
    msg.path_to_file = script_path + 'wait.png'
    turtlebro_controller.display_driver_srv_PlayMedia(msg)

    msg = EarsSetAngleRequest()
    msg.left_ear_angle = 0
    msg.right_ear_angle = 0
    turtlebro_controller.ears_driver_srv_EarsSetAngle(msg)

    msg = NeckSetAngleRequest()
    msg.horizontal_angle = 0
    msg.vertical_angle = 0
    msg.duration = 1
    msg.is_blocking = 1
    turtlebro_controller.neck_driver_srv_NeckSetAngle(msg)

    msg = PlayAudioRequest()
    msg.path_to_file = '__BLANK__'
    msg.is_blocking = 0
    msg.is_cycled = 0
    turtlebro_controller.speakers_driver_srv_PlayAudio(msg)

    msg = Twist()
    turtlebro_controller.turtlebro_pub_cmd_vel.publish(msg)