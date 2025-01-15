from turtlebot_controller_actions.main import *

import math

def run(turtlebot_controller:TurtlebotController): # Обязательно наличие этой функции, именно она вызывается при голосовой команде
    script_path = os.path.dirname(os.path.abspath(__file__)) + '/'
    
    msg = PlayMediaRequest()
    msg.is_blocking = 0
    msg.is_cycled = 0
    msg.path_to_file = script_path + 'move_away.png'
    turtlebot_controller.display_driver_srv_PlayMedia(msg)

    msg = PlayAudioRequest()
    msg.path_to_file = script_path + 'move_away_start.mp3'
    msg.is_blocking = 0
    msg.is_cycled = 0
    turtlebot_controller.speakers_driver_srv_PlayAudio(msg)

    msg = NeckSetAngleRequest()
    msg.horizontal_angle = 0
    msg.vertical_angle = 30
    msg.duration = 1
    msg.is_blocking = 0
    turtlebot_controller.neck_driver_srv_NeckSetAngle(msg)

    msg = EarsSetAngleRequest()
    msg.left_ear_angle = 0
    msg.right_ear_angle = -30
    turtlebot_controller.ears_driver_srv_EarsSetAngle(msg)

    delta = 0
    prev_theta = turtlebot_controller.turtlebot_odom_pose2d.theta
    cur_delta = 0

    msg = Twist()
    msg.angular.z = -0.7
    turtlebot_controller.turtlebot_pub_cmd_vel.publish(msg)
    while delta<math.pi:
        cur_theta = turtlebot_controller.turtlebot_odom_pose2d.theta
        cur_delta = prev_theta-cur_theta
        if (prev_theta<0) and (cur_theta>0):
            cur_delta += 2*math.pi
        if prev_theta != cur_theta:
            prev_theta = cur_theta
        delta += cur_delta

    msg.angular.z = 0
    turtlebot_controller.turtlebot_pub_cmd_vel.publish(msg)


    start_xy = (turtlebot_controller.turtlebot_odom_pose2d.x, turtlebot_controller.turtlebot_odom_pose2d.y)
    delta = math.sqrt((turtlebot_controller.turtlebot_odom_pose2d.x-start_xy[0])**2+(turtlebot_controller.turtlebot_odom_pose2d.y-start_xy[1])**2)

    msg = Twist()
    msg.linear.x = 0.2
    turtlebot_controller.turtlebot_pub_cmd_vel.publish(msg)
    while delta<0.2:
        delta = math.sqrt((turtlebot_controller.turtlebot_odom_pose2d.x-start_xy[0])**2+(turtlebot_controller.turtlebot_odom_pose2d.y-start_xy[1])**2)
        
    msg.linear.x = 0
    turtlebot_controller.turtlebot_pub_cmd_vel.publish(msg)

    msg = PlayAudioRequest()
    msg.path_to_file = script_path + 'move_away_finish.mp3'
    msg.is_blocking = 1
    msg.is_cycled = 0
    turtlebot_controller.speakers_driver_srv_PlayAudio(msg)