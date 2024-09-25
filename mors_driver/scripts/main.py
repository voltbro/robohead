# -*- coding: utf-8 -*-


import rospy
from geometry_msgs.msg import Twist, Pose, PoseArray
from trajectory_msgs.msg import JointTrajectoryPoint
from std_msgs.msg import Bool

from mors_driver.srv import TwistDuration, PoseArrayDuration, JointTrajectoryPointDuration
from mors_driver.srv import TwistDurationRequest, PoseArrayDurationRequest, JointTrajectoryPointDurationRequest
from mors_driver.srv import TwistDurationResponse, PoseArrayDurationResponse, JointTrajectoryPointDurationResponse
from mors_driver.srv import JointsCmdDuration, JointsCmdDurationResponse, JointsCmdDurationRequest

from mors.srv import QuadrupedCmd, QuadrupedCmdRequest, QuadrupedCmdResponse # Необходим пакет mors из mors_base v2.1
from mors.srv import JointsCmd, JointsCmdRequest, JointsCmdResponse # Необходим пакет mors из mors_base v2.1

import threading
import time

class MorsDriver():
    def __init__(self, node_name:str="mors_driver_node",
                 srv_vel_name:str="SetMorsVel", topic_vel_name:str="head/cmd_vel",
                 srv_pos_name:str="SetMorsPos", topic_pos_name:str="head/cmd_pose",
                 srv_ef_pos_name:str="SetMorsEfPos", topic_ef_pos_name:str="head/ef_position/command",
                 srv_joint_pos_name:str="SetMorsJointPos", topic_joint_pos_name:str="head/joint_group_position_controller/command",
                 srv_robot_action_name_out:str="robot_action", srv_robot_mode_name_out:str="robot_mode",
                 srv_robot_action_name_in:str="SetMorsAction", srv_robot_mode_name_in:str="SetMorsMode",
                 srv_joints_kp_name_out:str="joints_kp", srv_joints_kd_name_out:str="joints_kd",
                 srv_joints_kp_name_in:str="SetMorsJointsKp", srv_joints_kd_name_in:str="SetMorsJointsKd",
                 srv_stride_height_in:str="SetMorsStrideHeight", srv_stride_height_out:str="stride_height",
                 topic_status_name:str="head/status") -> None:

        rospy.init_node(node_name)

        self.pub_vel = rospy.Publisher(topic_vel_name, Twist, queue_size=10)
        self.pub_pos = rospy.Publisher(topic_pos_name, Twist, queue_size=10)
        self.pub_ef_pos = rospy.Publisher(topic_ef_pos_name, PoseArray, queue_size=10)
        self.pub_joint_pos = rospy.Publisher(topic_joint_pos_name, JointTrajectoryPoint, queue_size=10)
        self.pub_status = rospy.Publisher(topic_status_name, Bool, queue_size=10)

        self.srv_robot_action = rospy.ServiceProxy(srv_robot_action_name_out, QuadrupedCmd) # cmd: 1 - встать, 2 -лечь (подготовка к выключению), 3 - дать лапу, 4 - кувырок, 5 - помахать лапой.
        self.srv_robot_mode = rospy.ServiceProxy(srv_robot_mode_name_out, QuadrupedCmd) # cmd: 0 - ходьба, 1 - управление стопами, 2 - управление корпусом, 3 - управление сервоприводами.
        self.srv_robot_joints_kp = rospy.ServiceProxy(srv_joints_kp_name_out, JointsCmd)
        self.srv_robot_joints_kd = rospy.ServiceProxy(srv_joints_kd_name_out, JointsCmd)
        self.srv_stride_height = rospy.ServiceProxy(srv_stride_height_out, QuadrupedCmd)

        rospy.Service(srv_vel_name, TwistDuration, self.__server_vel)
        rospy.Service(srv_pos_name, TwistDuration, self.__server_pos)
        rospy.Service(srv_ef_pos_name, PoseArrayDuration, self.__server_ef_pos)
        rospy.Service(srv_joint_pos_name, JointTrajectoryPointDuration, self.__server_joint_pos)
        rospy.Service(srv_robot_mode_name_in, QuadrupedCmd, self.__server_set_robot_mode)
        rospy.Service(srv_robot_action_name_in, QuadrupedCmd, self.__server_do_action)
        rospy.Service(srv_joints_kp_name_in, JointsCmd, self.__server_joints_kp)
        rospy.Service(srv_joints_kd_name_in, JointsCmd, self.__server_joints_kd)
        rospy.Service(srv_stride_height_in, QuadrupedCmd, self.__server_stride_height)        

        cur_time = time.time()

        self.goal_tuple = (cur_time, None, 0)
        self.robot_mode = 0
        self.prev_goal = None
        self.rate = rospy.Rate(300)

        self.isStandup = False
        t1 = threading.Thread(target=self.__executer, daemon=True)
        t1.start()
        
        rospy.spin()
    
    def __executer(self):
        prev_stamp=0
        while True:
            self.pub_status.publish(True)
            robot_mode = self.robot_mode

            if (robot_mode==0): # ходьба 0
                stamp, goal, dur = self.goal_tuple

                if (type(goal)!=Twist):
                    self._set_vel(Twist())
                    continue

                if (stamp>prev_stamp):
                    prev_stamp = stamp
                    if (self.prev_goal==None):
                        self.prev_goal = Twist()
                    else:
                        self.prev_goal = tmp
                    tmp = goal
                    start_time=time.time()
                
                cur_time = time.time()
                cmd = Twist()
                cmd.angular.x = self.__generate_val(cur=self.prev_goal.angular.x, goal=goal.angular.x, duration=dur, start_time=start_time, current_time=cur_time)
                cmd.angular.y = self.__generate_val(cur=self.prev_goal.angular.y, goal=goal.angular.y, duration=dur, start_time=start_time, current_time=cur_time)
                cmd.angular.z = self.__generate_val(cur=self.prev_goal.angular.z, goal=goal.angular.z, duration=dur, start_time=start_time, current_time=cur_time)
                cmd.linear.x = self.__generate_val(cur=self.prev_goal.linear.x, goal=goal.linear.x, duration=dur, start_time=start_time, current_time=cur_time)
                cmd.linear.y = self.__generate_val(cur=self.prev_goal.linear.y, goal=goal.linear.y, duration=dur, start_time=start_time, current_time=cur_time)
                cmd.linear.z = self.__generate_val(cur=self.prev_goal.linear.z, goal=goal.linear.z, duration=dur, start_time=start_time, current_time=cur_time)
                
                self._set_vel(cmd)
            
            elif (robot_mode==1): # управление стопами лап 1
                stamp, goal, dur = self.goal_tuple
                if (type(goal)!=PoseArray):
                    self._set_ef_pos(PoseArray())
                    continue
            
                if (stamp>prev_stamp):
                    prev_stamp = stamp
                    if (self.prev_goal==None):
                        self.prev_goal = PoseArray()
                        self.prev_goal.poses = [Pose()]*4
                    else:
                        self.prev_goal = tmp
                    tmp = goal
                    start_time=time.time()

                cur_time = time.time()
                cmd = PoseArray()

                for i in range(4):
                    pose = Pose()
                    pose.position.x = self.__generate_val(cur=self.prev_goal.poses[i].position.x, goal=goal.poses[i].position.x, duration=dur, start_time=start_time, current_time=cur_time)
                    pose.position.y = self.__generate_val(cur=self.prev_goal.poses[i].position.y, goal=goal.poses[i].position.y, duration=dur, start_time=start_time, current_time=cur_time)
                    pose.position.z = self.__generate_val(cur=self.prev_goal.poses[i].position.z, goal=goal.poses[i].position.z, duration=dur, start_time=start_time, current_time=cur_time)
                    pose.orientation.x = self.__generate_val(cur=self.prev_goal.poses[i].orientation.x, goal=goal.poses[i].orientation.x, duration=dur, start_time=start_time, current_time=cur_time)
                    pose.orientation.y = self.__generate_val(cur=self.prev_goal.poses[i].orientation.y, goal=goal.poses[i].orientation.y, duration=dur, start_time=start_time, current_time=cur_time)
                    pose.orientation.z = self.__generate_val(cur=self.prev_goal.poses[i].orientation.z, goal=goal.poses[i].orientation.z, duration=dur, start_time=start_time, current_time=cur_time)
                    pose.orientation.w = self.__generate_val(cur=self.prev_goal.poses[i].orientation.w, goal=goal.poses[i].orientation.w, duration=dur, start_time=start_time, current_time=cur_time)
                    cmd.poses.append(pose)
                
                self._set_ef_pos(cmd)

            elif (robot_mode==2): # управление корпусом 2
                stamp, goal, dur = self.goal_tuple
                if (type(goal)!=Twist):
                    self._set_pos(Twist())
                    continue

                if (stamp>prev_stamp):
                    prev_stamp = stamp
                    if (self.prev_goal==None):
                        self.prev_goal = Twist()
                    else:
                        self.prev_goal = tmp
                    tmp = goal
                    start_time=time.time()
                
                cur_time = time.time()
                cmd = Twist()
                cmd.angular.x = self.__generate_val(cur=self.prev_goal.angular.x, goal=goal.angular.x, duration=dur, start_time=start_time, current_time=cur_time)
                cmd.angular.y = self.__generate_val(cur=self.prev_goal.angular.y, goal=goal.angular.y, duration=dur, start_time=start_time, current_time=cur_time)
                cmd.angular.z = self.__generate_val(cur=self.prev_goal.angular.z, goal=goal.angular.z, duration=dur, start_time=start_time, current_time=cur_time)
                cmd.linear.x = self.__generate_val(cur=self.prev_goal.linear.x, goal=goal.linear.x, duration=dur, start_time=start_time, current_time=cur_time)
                cmd.linear.y = self.__generate_val(cur=self.prev_goal.linear.y, goal=goal.linear.y, duration=dur, start_time=start_time, current_time=cur_time)
                cmd.linear.z = self.__generate_val(cur=self.prev_goal.linear.z, goal=goal.linear.z, duration=dur, start_time=start_time, current_time=cur_time)
                
                self._set_pos(cmd)

            elif (robot_mode==3): # управление сервоприводами 3
                stamp, goal, dur = self.goal_tuple
                if (type(goal)!=JointTrajectoryPoint):
                    self._set_joint_pos(JointTrajectoryPoint())
                    continue

                if (stamp>prev_stamp):
                    prev_stamp = stamp
                    if (self.prev_goal==None):
                        self.prev_goal = JointTrajectoryPoint()
                        self.prev_goal.positions = [0]*12
                        self.prev_goal.velocities = [0]*12
                        self.prev_goal.accelerations = [0]*12
                        self.prev_goal.effort = [0]*12
                    else:
                        self.prev_goal = tmp
                    tmp = goal
                    start_time=time.time()
                
                cur_time = time.time()
                cmd = JointTrajectoryPoint()
                for i in range(12):
                    cmd.positions.append(self.__generate_val(cur=self.prev_goal.positions[i], goal=goal.positions[i], duration=dur, start_time=start_time, current_time=cur_time))
                    cmd.velocities.append(self.__generate_val(cur=self.prev_goal.velocities[i], goal=goal.velocities[i], duration=dur, start_time=start_time, current_time=cur_time))
                    cmd.accelerations.append(self.__generate_val(cur=self.prev_goal.accelerations[i], goal=goal.accelerations[i], duration=dur, start_time=start_time, current_time=cur_time))
                    cmd.effort.append(self.__generate_val(cur=self.prev_goal.effort[i], goal=goal.effort[i], start_time=start_time, duration=dur, current_time=cur_time))
                
                self._set_joint_pos(cmd)
            self.rate.sleep()
    
    def __server_vel(self, request:TwistDurationRequest):
        self.goal_tuple = (time.time(), request.data, request.duration)
        return TwistDurationResponse(0)
    
    def __server_vel(self, request:TwistDurationRequest):
        self.goal_tuple = (time.time(), request.data, request.duration)
        return TwistDurationResponse(0)
    
    def __server_pos(self, request:TwistDurationRequest):
        self.goal_tuple = (time.time(), request.data, request.duration)
        return TwistDurationResponse(0)
    
    def __server_ef_pos(self, request:PoseArrayDurationRequest):
        self.goal_tuple = (time.time(), request.data, request.duration)
        return PoseArrayDurationResponse(0)

    def __server_joint_pos(self, request:JointTrajectoryPointDurationRequest):
        self.goal_tuple = (time.time(), request.data, request.duration)
        return JointTrajectoryPointDurationResponse(0)
    
    def __server_joints_kp(self, request:JointsCmdRequest):
        self.srv_robot_joints_kp(request.cmd)
        return JointsCmdResponse(0,"")

    def __server_joints_kd(self, request:JointsCmdRequest):
        self.srv_robot_joints_kd(request.cmd)
        return JointsCmdResponse(0,"")
    
    def __server_stride_height(self, request:QuadrupedCmd):
        self.srv_stride_height(request)
        return QuadrupedCmdResponse(0, "")

    
    def _set_vel(self, vel:Twist) -> None:
        self.pub_vel.publish(vel)
    
    def _set_pos(self, pos:Twist) -> None:
        self.pub_pos.publish(pos)
    
    def _set_ef_pos(self, ef_pos:PoseArray) -> None:
        self.pub_ef_pos.publish(ef_pos)
    
    def _set_joint_pos(self, joint_pos:JointTrajectoryPoint) -> None:
        self.pub_joint_pos.publish(joint_pos)
    
    def __server_set_robot_mode(self, request:QuadrupedCmdRequest) -> None:
        self.srv_robot_mode(request)
        self.prev_goal = None
        self.goal_tuple = (time.time(), None, 0)
        self.robot_mode = request.cmd
        return QuadrupedCmdResponse(0, "")

    def __server_do_action(self, request:QuadrupedCmdRequest) -> None:
        if request.cmd==4:
            return QuadrupedCmdResponse(1, "Blocked Action!")
        
        if  (request.cmd==1) and (self.isStandup==True):
            return QuadrupedCmdResponse(1, "Already standup")
        
        if  (request.cmd==2) and (self.isStandup==False):
            return QuadrupedCmdResponse(1, "Already lay")
        
        if request.cmd == 1:
            self.isStandup = True
        elif request.cmd==2:
            self.isStandup = False

        self.srv_robot_action(request)
        return QuadrupedCmdResponse(0, "")
            


    def __generate_val(self, cur:float=None, goal:float=None, duration:float=1.0, start_time:float=None, current_time:float=None):
        t = current_time-start_time

        if ((duration!=0) and (t<=duration)):
            cubic = self.__generate_cubic(cur, goal, duration)
            return cubic[0]+cubic[2]*(t**2)+cubic[3]*(t**3)
        else:
            return goal

    def __generate_cubic(self, cur:float=None, goal:float=None, duration:float=1.0):
        a0 = cur
        a1 = 0
        a2 = (3/(duration**2))*(goal-cur)
        a3 = (-2/(duration**3))*(goal-cur)
        return (a0, a1, a2, a3)

if __name__ == "__main__":
    MorsDriver()