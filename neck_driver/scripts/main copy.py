from adafruit_servokit import ServoKit
import rospy
from neck_driver.srv import NeckSetAngle, NeckSetAngleResponse, NeckSetAngleRequest
import multiprocessing

class NeckDriver():
    def __init__(self)->None:
        self._kit = ServoKit(channels=16)

        srv_name = rospy.get_param("~service_name", "~NeckSetAngle")
        std_vertical_angle = rospy.get_param("~std_vertical_angle", 0)
        std_horizontal_angle = rospy.get_param("~std_horizontal_angle", 0)

        self._servo_1_channel = rospy.get_param("~servo_1_channel", 9)
        self._servo_2_channel = rospy.get_param("~servo_2_channel", 8)

        self._servo_1_coef = rospy.get_param("~servo_1_coef", 0)
        self._servo_2_coef = rospy.get_param("~servo_2_coef", 0)

        self._constraint_v_from = rospy.get_param("~constraints/v_from", -30)
        self._constraint_v_to = rospy.get_param("~constraints/v_to", 30)
        self._constraint_h_from = rospy.get_param("~constraints/h_from", -30)
        self._constraint_h_to = rospy.get_param("~constraints/h_to", 30)

        manager = multiprocessing.Manager()
        self._goal_response = manager.Value('goal_response', (std_vertical_angle, std_horizontal_angle, 0))
        self._current_angles = manager.Value('curent_angles', (std_vertical_angle, std_horizontal_angle))

        self.set_angle(vertical=std_vertical_angle, horizontal=std_horizontal_angle)

        process = multiprocessing.Process(target=self._trajectory_planner, args=(), daemon=True)
        process.start()

        rospy.Service(srv_name, NeckSetAngle, self._requester)
        rospy.loginfo("neck_driver INITED")

    def _trajectory_planner(self):
        prev_goal_v, prev_goal_h = self._current_angles.value
        start_time = rospy.get_time()

        while True:
            cur_v, cur_h = self._current_angles.value
            goal_v, goal_h, duration = self._goal_response.value
            print("goal", goal_h, goal_v)

            if duration!=0:
                if (prev_goal_v!=goal_v) or (prev_goal_h!=goal_h):
                    start_time = rospy.get_time()
                    cubic_v = self.__generate_cubic(angle_cur=cur_v, angle_goal=goal_v, time=duration)
                    cubic_h = self.__generate_cubic(angle_cur=cur_h, angle_goal=goal_h, time=duration)
                    prev_goal_v = goal_v
                    prev_goal_h = goal_h

                if ( (((goal_v-cur_v)**2+(goal_h-cur_h)**2)**(1/2)) >= 0.5) and ((rospy.get_time()-start_time)<duration):
                    t = rospy.get_time()-start_time
                    thetha_v = cubic_v[0]+cubic_v[2]*(t**2)+cubic_v[3]*(t**3)
                    thetha_h = cubic_h[0]+cubic_h[2]*(t**2)+cubic_h[3]*(t**3)
                    self.set_angle(vertical=thetha_v, horizontal=thetha_h)
                else:
                    if (goal_v!=cur_v) or (goal_h!=cur_h):
                        self.set_angle(vertical=goal_v, horizontal=goal_h)
            else:
                if (prev_goal_v!=goal_v) or (prev_goal_h!=goal_h):
                    self.set_angle(vertical=goal_v, horizontal=goal_h)
                    prev_goal_v = goal_v
                    prev_goal_h = goal_h


    def __generate_cubic(self, angle_cur:int=None, angle_goal:int=None, time:float=1.0):
        a0 = angle_cur
        a1 = 0
        a2 = (3/(time**2))*(angle_goal-angle_cur)
        a3 = (-2/(time**3))*(angle_goal-angle_cur)
        return (a0, a1, a2, a3)
    
    def _requester(self, request:NeckSetAngleRequest):
        response = NeckSetAngleResponse()
        print("request:",request.horizontal_angle, request.vertical_angle)
        if self._constraint_v_from<=request.vertical_angle<=self._constraint_v_to:
            if self._constraint_h_from<=request.horizontal_angle<=self._constraint_h_to:
                if request.duration>=0:
                    self._goal_response.value = (request.vertical_angle, request.horizontal_angle, request.duration)
                    response.value = 0
                else:
                    response.value = -3
            else:
                response.value = -2
        else:
            response.value = -1
        
        if request.is_blocking:
            while (self._current_angles.value!=self._goal_response.value[0:2]):
                rospy.sleep(0.05)
        return response
    
    def set_angle(self, vertical:int=None, horizontal:int=None) -> None:
        if vertical!=None:
            self.__vertical = vertical
        if horizontal!=None:
            self.__horizontal = horizontal
        
        self._kit.servo[self._servo_1_channel].angle = 90-self.__vertical-self.__horizontal+self._servo_1_coef
        self._kit.servo[self._servo_2_channel].angle = 90+self.__vertical-self.__horizontal+self._servo_2_coef

        self._current_angles.value = (self.__vertical, self.__horizontal)

if __name__ == "__main__":
    rospy.init_node("neck_driver")
    obj = NeckDriver()
    rospy.spin()