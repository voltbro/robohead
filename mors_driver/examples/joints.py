# ------------------------------------------------------------
# Файл содержит пример кода управления двигателями
# шагающего робота МОРС. Во время демонстрационного примера 
# робот вращает суставами по синусоидальной траектории с 
# разными амплитудами для каждого сустава. 
# Обязательно поставьте робота на стойку перед тем, как 
# запускать это пример. Будьте аккуратны, так как робот
# достаточно сильно размахивает ногами.
# Запуск: rosrun mors_demo demo_joints.py
# ------------------------------------------------------------

import rospy
import time

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from mors.srv import QuadrupedCmd, QuadrupedCmdResponse
from mors_driver.srv import JointTrajectoryPointDuration, JointTrajectoryPointDurationRequest
from mors.srv import QuadrupedCmd, JointsCmd

def set_mode_client(mode:int) -> QuadrupedCmdResponse:
    rospy.wait_for_service('SetMorsMode')
    try:
        set_mode = rospy.ServiceProxy('SetMorsMode', QuadrupedCmd)
        resp = set_mode(mode)
        return resp.result
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


# call joints_kpkd service
def set_joints_kp(kp):
    rospy.wait_for_service("SetMorsJointsKp")
    try:
        set_kp_srv = rospy.ServiceProxy('SetMorsJointsKp', JointsCmd)
        resp = set_kp_srv(kp)
        return resp.result
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def set_joints_kd(kd):
    rospy.wait_for_service("SetMorsJointsKp")
    try:
        set_kd_srv = rospy.ServiceProxy('SetMorsJointsKd', JointsCmd)
        resp = set_kd_srv(kd)
        return resp.result
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

if __name__ == '__main__':
    # инициализация ROS
    rospy.init_node("mors_demo")
    srv_goal_joint_pos = rospy.ServiceProxy('SetMorsJointPos', JointTrajectoryPointDuration)

    cmd_joint_msg = JointTrajectoryPoint()
    cmd_joint_msg.velocities = [0]*12
    cmd_joint_msg.effort = [0]*12
    cmd_joint_msg.positions = [0]*12
    cmd_joint_msg.accelerations = [0]*12
 
    rospy.loginfo("Demo Joints: Start")
    # обнуляем коэффициенты обратной связи для моторов
    set_joints_kp([0.0]*12)
    set_joints_kd([0.0]*12)

    # устанавливаем режим управления двигателями
    set_mode_client(3)

    # постепенно увеличиваем коэффициенты обратной связи для моторов,
    # чтобы выпрямить ноги
    rospy.sleep(0.2)
    set_joints_kp([0.2]*12)
    set_joints_kd([0.2]*12)
    rospy.sleep(1.5)
    set_joints_kp([2.0]*12)
    set_joints_kd([0.2]*12)
    rospy.sleep(1.5)
    set_joints_kp([16.0]*12)
    set_joints_kd([0.4]*12)
    rospy.sleep(1.5)

    k = 1
    for i in range(4):
        print(i, k)
        theta1 = 0.4*k
        theta2 = 1.0*k
        theta3 = 1.57*k

        cmd_joint_msg.positions = [theta1, theta2, theta3,
                                    theta1, theta2, theta3,
                                    theta1, theta2, theta3,
                                    theta1, theta2, theta3]
        
        msg = JointTrajectoryPointDurationRequest()
        msg.data = cmd_joint_msg
        msg.duration = 2
        srv_goal_joint_pos(msg)
        time.sleep(2)
        k = k*(-1)

    # обнуляем коэффициенты
    set_joints_kp([0.0]*12)
    set_joints_kd([0.2]*12)
    rospy.sleep(2.0)
    set_joints_kp([0.0]*12)
    set_joints_kd([0.0]*12)
    rospy.loginfo("Demo Joints: Stop")
    set_mode_client(0)