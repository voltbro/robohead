from turtlebro_controller_actions.main import *

import math

def run(turtlebro_controller:TurtlebroController, cmds:str): # Обязательно наличие этой функции, именно она вызывается при голосовой команде
    script_path = os.path.dirname(os.path.abspath(__file__)) + '/'
    
    msg = PlayMediaRequest()
    msg.is_blocking = 0
    msg.is_cycled = 0
    msg.path_to_file = script_path + 'move_here.png'
    turtlebro_controller.display_driver_srv_PlayMedia(msg)

    msg = PlayAudioRequest()
    msg.path_to_file = script_path + 'move_here_start.mp3'
    msg.is_blocking = 0
    msg.is_cycled = 0
    turtlebro_controller.speakers_driver_srv_PlayAudio(msg)

    msg = NeckSetAngleRequest()
    msg.horizontal_angle = 0
    msg.vertical_angle = 30
    msg.duration = 1
    msg.is_blocking = 0
    turtlebro_controller.neck_driver_srv_NeckSetAngle(msg)

    msg = EarsSetAngleRequest()
    msg.left_ear_angle = 0
    msg.right_ear_angle = -30
    turtlebro_controller.ears_driver_srv_EarsSetAngle(msg)

    start_xy = (turtlebro_controller.turtlebro_odom_pose2d.x, turtlebro_controller.turtlebro_odom_pose2d.y)

    delta = math.sqrt((turtlebro_controller.turtlebro_odom_pose2d.x-start_xy[0])**2+(turtlebro_controller.turtlebro_odom_pose2d.y-start_xy[1])**2)

    msg = Twist()
    msg.linear.x = 0.2
    turtlebro_controller.turtlebro_pub_cmd_vel.publish(msg)
    while delta<0.5:
        delta = math.sqrt((turtlebro_controller.turtlebro_odom_pose2d.x-start_xy[0])**2+(turtlebro_controller.turtlebro_odom_pose2d.y-start_xy[1])**2)
        
    msg.linear.x = 0
    turtlebro_controller.turtlebro_pub_cmd_vel.publish(msg)

    msg = PlayAudioRequest()
    msg.path_to_file = script_path + 'move_here_finish.mp3'
    msg.is_blocking = 1
    msg.is_cycled = 0
    turtlebro_controller.speakers_driver_srv_PlayAudio(msg)

            
        
        




    