from adafruit_servokit import ServoKit
import time
import rospy
from ears_controller.srv import EarsSetAngle, EarsSetAngleResponse
class NeckController():
    def __init__(self, node_name:str="ears_controller_node", std_left_angle:int=0, std_right_angle:int=0,
                 servo1_channel:int=11, servo2_channel:int=10, service_name:str="EarsSetAngle")->None:
        self.__kit = ServoKit(channels=16)
        self.__servo1 = servo1_channel
        self.__servo2 = servo2_channel
        
        self.set_angle(left=std_left_angle, right=std_right_angle)
        rospy.init_node(node_name)
        rospy.Service(service_name, EarsSetAngle, self.__requester)

        rospy.spin()
        
    def __requester(self, request:EarsSetAngle):
        self.set_angle(request.EarLeft, request.EarRight)
        return EarsSetAngleResponse(0)
    
    def set_angle(self, left:int=None, right:int=None) -> None:
        if left!=None:
            self.__left = left
        if right!=None:
            self.__right = right
        
        self.__kit.servo[self.__servo1].angle = 90+self.__left
        self.__kit.servo[self.__servo2].angle = 90-self.__right

if __name__ == "__main__":
    NeckController()