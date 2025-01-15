from turtlebot_controller_actions.main import *

def run(robohead_controller:TurtlebotController): # Обязательно наличие этой функции, именно она вызывается при голосовой команде
    script_path = os.path.dirname(os.path.abspath(__file__)) + '/'
    
    msg = PlayMediaRequest()
    msg.is_blocking = 0
    msg.is_cycled = 0
    msg.path_to_file = script_path + 'left_ear.png'
    robohead_controller.display_driver_srv_PlayMedia(msg)

    msg = PlayAudioRequest()
    msg.path_to_file = script_path + 'left_ear.mp3'
    msg.is_blocking = 0
    msg.is_cycled = 0
    robohead_controller.speakers_driver_srv_PlayAudio(msg)

    msg = NeckSetAngleRequest()
    msg.horizontal_angle = 0
    msg.vertical_angle = 30
    msg.duration = 1
    msg.is_blocking = 0
    robohead_controller.neck_driver_srv_NeckSetAngle(msg)

    msg = EarsSetAngleRequest()
    msg.left_ear_angle = -30
    msg.right_ear_angle = 0
    robohead_controller.ears_driver_srv_EarsSetAngle(msg)
    
    k=-1
    for _ in range(5):
        rospy.sleep(0.5)
        msg.left_ear_angle = -30*k
        k*=-1
        robohead_controller.ears_driver_srv_EarsSetAngle(msg)




    