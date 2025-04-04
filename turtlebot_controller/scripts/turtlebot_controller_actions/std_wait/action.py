from turtlebot_controller_actions.main import *

def run(turtlebot_controller:TurtlebotController, cmds:str): # Обязательно наличие этой функции, именно она вызывается при голосовой команде
    script_path = os.path.dirname(os.path.abspath(__file__)) + '/'
    
    msg = PlayMediaRequest()
    msg.is_blocking = 0
    msg.is_cycled = 0
    msg.path_to_file = script_path + 'wait.png'
    turtlebot_controller.display_driver_srv_PlayMedia(msg)

    msg = EarsSetAngleRequest()
    msg.left_ear_angle = 0
    msg.right_ear_angle = 0
    turtlebot_controller.ears_driver_srv_EarsSetAngle(msg)

    msg = NeckSetAngleRequest()
    msg.horizontal_angle = 0
    msg.vertical_angle = 0
    msg.duration = 1
    msg.is_blocking = 1
    turtlebot_controller.neck_driver_srv_NeckSetAngle(msg)

    msg = PlayAudioRequest()
    msg.path_to_file = '__BLANK__'
    msg.is_blocking = 0
    msg.is_cycled = 0
    turtlebot_controller.speakers_driver_srv_PlayAudio(msg)

    msg = Twist()
    turtlebot_controller.turtlebot_pub_cmd_vel.publish(msg)