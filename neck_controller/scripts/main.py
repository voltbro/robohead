from adafruit_servokit import ServoKit
import rospy
from neck_controller.srv import NeckSetAngle, NeckSetAngleResponse
import time
import multiprocessing
import math

class NeckController():
    def __init__(self, node_name:str="neck_controller_node", std_vertical_angle:int=0, std_horizontal_angle:int=0,
                 servo1_channel:int=8, servo2_channel:int=9, service_name:str="NeckSetAngle")->None:
        self.__kit = ServoKit(channels=16)
        self.__servo1 = servo1_channel
        self.__servo2 = servo2_channel

        manager = multiprocessing.Manager()
        self.goal_angles_and_duration = manager.Value('goal_angles', (std_vertical_angle, std_horizontal_angle, 1))
        self.current_angles = manager.Value('curent_angles', (std_vertical_angle, std_horizontal_angle))
        play = multiprocessing.Process(target=self._mover, args=(), daemon=True)
        play.start()

        self.set_angle(vertical=std_vertical_angle, horizontal=std_horizontal_angle)
        self._last_angle_v = std_vertical_angle
        self._last_angle_h = std_horizontal_angle


        rospy.init_node(node_name)
        rospy.Service(service_name, NeckSetAngle, self.__requester)

        rospy.spin()
    
    def _mover(self):
        prev_goal_v = 0
        prev_goal_h = 0

        while True:
            cur_v, cur_h = self.current_angles.value
            goal_v, goal_h, duration = self.goal_angles_and_duration.value

            if duration!=0:
                if (prev_goal_v!=goal_v) or (prev_goal_h!=goal_h):
                    prev_goal_v = goal_v
                    prev_goal_h = goal_h
                    start_time = time.time()
                    cubic_v = self.__generate_cubic(angle_cur=cur_v, angle_goal=goal_v, time=duration)
                    cubic_h = self.__generate_cubic(angle_cur=cur_h, angle_goal=goal_h, time=duration)

                if (math.sqrt((goal_v-cur_v)**2+(goal_h-cur_h)**2)>=0.5) and ((time.time()-start_time)<duration):
                    t = time.time()-start_time
                    thetha_v = cubic_v[0]+cubic_v[2]*(t**2)+cubic_v[3]*(t**3)
                    thetha_h = cubic_h[0]+cubic_h[2]*(t**2)+cubic_h[3]*(t**3)
                    self.set_angle(vertical=thetha_v, horizontal=thetha_h)
                else:
                    if (goal_v!=cur_v) or (goal_h!=cur_h):
                        self.set_angle(vertical=goal_v, horizontal=goal_h)
            else:
                self.set_angle(vertical=goal_v, horizontal=goal_h)


    def __requester(self, request:NeckSetAngle):
        self.goal_angles_and_duration.value = (request.AngleV, request.AngleH, request.TimeF)
        return NeckSetAngleResponse(0)
    
    def set_angle(self, vertical:int=None, horizontal:int=None) -> None:
        if vertical!=None:
            self.__vertical = vertical
        if horizontal!=None:
            self.__horizontal = horizontal
        
        self.__kit.servo[self.__servo1].angle = 90-self.__vertical-self.__horizontal
        self.__kit.servo[self.__servo2].angle = 90+self.__vertical-self.__horizontal

        self.current_angles.value = (vertical, horizontal)
    
    def __generate_cubic(self, angle_cur:int=None, angle_goal:int=None, time:float=1.0):
        a0 = angle_cur
        a1 = 0
        a2 = (3/(time**2))*(angle_goal-angle_cur)
        a3 = (-2/(time**3))*(angle_goal-angle_cur)
        return (a0, a1, a2, a3)


if __name__ == "__main__":
    NeckController()
