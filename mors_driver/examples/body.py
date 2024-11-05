# ------------------------------------------------------------
# Файл содержит пример кода управления корпусом
# шагающего робота МОРС
# Запуск: rosrun mors_demo demo_body.py
# ------------------------------------------------------------

import rospy
import time

from mors_driver.srv import TwistDuration, TwistDurationRequest
from geometry_msgs.msg import Twist

from mors.srv import QuadrupedCmd, QuadrupedCmdResponse

# call robot_mode service
def set_mode_client(mode:int) -> QuadrupedCmdResponse:
    rospy.wait_for_service('SetMorsMode')
    try:
        set_mode = rospy.ServiceProxy('SetMorsMode', QuadrupedCmd)
        resp = set_mode(mode)
        return resp.result
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

# call robot_action service
def set_action_client(action:int) -> QuadrupedCmdResponse:
    rospy.wait_for_service('SetMorsAction')
    try:
        set_action = rospy.ServiceProxy('SetMorsAction', QuadrupedCmd)
        resp = set_action(action)
        return resp.result
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


if __name__ == '__main__':
    # инициализация ROS
    rospy.init_node("example_mors_driver_body_node")
    srv_goal_pos = rospy.ServiceProxy('SetMorsPos', TwistDuration)

    # встаем
    set_action_client(1)

    # устанавливаем режим управления корпусом
    set_mode_client(2)

    request = TwistDurationRequest()
    
    # меняем положение корпуса вдоль вертикальной оси
    for i in range(2):
        request.data.linear.z = 0.05
        request.duration = 2
        srv_goal_pos(request)
        time.sleep(2)
        request.data.linear.z = -0.05
        request.duration = 2
        srv_goal_pos(request)
        time.sleep(2)
    
    # вращаем корпус по рысканью
    request.data.linear.z = 0 # Обнуляем предыдущее значение
    for i in range(2):
        request.data.angular.z = 0.4
        request.duration = 3
        srv_goal_pos(request)
        time.sleep(3)
        request.data.angular.z = -0.4
        request.duration = 3
        srv_goal_pos(request)
        time.sleep(3)
    
    # Возвращаемся в начальное положение
    request.data = Twist()
    request.duration = 2
    srv_goal_pos(request)
    time.sleep(2)
    set_action_client(2)