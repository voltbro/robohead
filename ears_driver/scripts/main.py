from adafruit_servokit import ServoKit
import rospy
from ears_driver.srv import EarsSetAngle, EarsSetAngleResponse

class EarsDriverPy():
    def __init__(self)->None:
        
        self._kit = ServoKit(channels=16)

        rospy.init_node("ears_driver")

        srv_name = rospy.get_param("~service_name", "~EarsSetAngle")
        std_L_angle = rospy.get_param("~std_L_angle", 0)
        std_R_angle = rospy.get_param("~std_R_angle", 0)

        self._servoL_chnl = rospy.get_param("~servo_L_channel", 11)
        self._servoR_chnl = rospy.get_param("~servo_R_channel", 10)

        self._constraint_L_from = rospy.get_param("~constraints/L_from", -90)
        self._constraint_L_to = rospy.get_param("~constraints/L_to", 90)
        self._constraint_R_from = rospy.get_param("~constraints/R_from", -90)
        self._constraint_R_to = rospy.get_param("~constraints/R_to", 90)

        self.set_angle(left_ear=std_L_angle, right_ear=std_R_angle)

        rospy.Service(srv_name, EarsSetAngle, self._requester)

        rospy.loginfo("ears_driver INITED")
        rospy.spin()
        
    def _requester(self, request:EarsSetAngle):
        response = EarsSetAngleResponse()
        if self._constraint_L_from<=request.left_ear_angle<=self._constraint_L_to:
            if self._constraint_R_from<=request.right_ear_angle<=self._constraint_R_to:
                self.set_angle(request.left_ear_angle, request.right_ear_angle)
                response.value = 0
            else:
                response.value = -2
        else:
            response.value = -1

        return response
    
    def set_angle(self, left_ear:int=None, right_ear:int=None) -> None:
        if left_ear!=None:
            self._left_ear = left_ear
        if right_ear!=None:
            self._right_ear = right_ear
        
        self._kit.servo[self._servoL_chnl].angle = 90+self._left_ear
        self._kit.servo[self._servoR_chnl].angle = 90-self._right_ear

if __name__ == "__main__":
    a = EarsDriverPy()