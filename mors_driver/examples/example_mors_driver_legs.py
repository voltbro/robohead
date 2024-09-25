# ------------------------------------------------------------
# Файл содержит пример кода управления стопами
# шагающего робота МОРС. Во время демонстрационного примера 
# робота двигает стопами по окружности. Каждая нога 
# отрабатывает движение в своей плоскости. Поставьте робота
# на стойку перед тем, как запускать пример.
# Запуск: rosrun mors_demo demo_legs.py
# ------------------------------------------------------------

import rospy
from geometry_msgs.msg import PoseArray, Pose
import time
from mors.srv import QuadrupedCmd, QuadrupedCmdResponse, QuadrupedCmdRequest
from mors_driver.srv import PoseArrayDuration, PoseArrayDurationRequest

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
    rospy.init_node("mors_demo")
    srv_goal_ef_pos = rospy.ServiceProxy('SetMorsEfPos', PoseArrayDuration)

    legs = PoseArray()
    legs.poses = [Pose()]*4
 
    rospy.loginfo("Demo Legs: Start")

    # встаем
    set_action_client(1)

    # устанавливаем режим управления ногами
    set_mode_client(1)
    
    k = 1
    for i in range(6):
        r1 = Pose()
        r1.position.x = 0.06*k
        r1.position.y = -0.06*k
        r1.position.z = 0.06*k

        r2 = Pose()
        r2.position.x = -0.06*k
        r2.position.y = -0.06*k
        r2.position.z = 0.06*k

        l1 = Pose()
        l1.position.x = 0.06*k
        l1.position.y = 0.06*k
        l1.position.z = 0.06*k

        l2 = Pose()
        l2.position.x = -0.06*k
        l2.position.y = 0.06*k
        l2.position.z = 0.06*k

        legs.poses[0] = r1
        legs.poses[1] = l1
        legs.poses[2] = r2
        legs.poses[3] = l2

        msg = PoseArrayDurationRequest()
        msg.data = legs
        msg.duration = 2
        srv_goal_ef_pos(msg)
        time.sleep(2.5)

        k = k*(-1)

    legs = PoseArray()
    legs.poses = [Pose()]*4
    msg = PoseArrayDurationRequest()
    msg.data = legs
    msg.duration = 1
    srv_goal_ef_pos(msg)
    

    
  
    set_action_client(2)
    rospy.loginfo("Demo Legs: Stop")