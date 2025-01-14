from robohead_controller_actions.main import *

import cv2
from cv_bridge import CvBridge
import numpy as np

def run(robohead_controller:RoboheadController): # Обязательно наличие этой функции, именно она вызывается при голосовой команде
    script_path = os.path.dirname(os.path.abspath(__file__)) + '/'

    msg = EarsSetAngleRequest()
    msg.left_ear_angle = -30
    msg.right_ear_angle = -30
    robohead_controller.ears_driver_srv_EarsSetAngle(msg)

    msg = NeckSetAngleRequest()
    msg.horizontal_angle = 0
    msg.vertical_angle = 30
    msg.duration = 1
    msg.is_blocking = 1
    robohead_controller.neck_driver_srv_NeckSetAngle(msg)

    msg = PlayAudioRequest()
    msg.path_to_file = script_path + 'make_photo.mp3'
    msg.is_blocking = 0
    msg.is_cycled = 0
    robohead_controller.speakers_driver_srv_PlayAudio(msg)

    rospy.sleep(0.5)
    cvBridge = CvBridge()
    cv_image = cvBridge.imgmsg_to_cv2(robohead_controller.cv_camera_image_raw, "bgr8")
    cv_image = cv2.resize(cv_image, (1080, 1080))
    robohead_controller.display_driver_pub_PlayMedia.publish(cvBridge.cv2_to_imgmsg(cv_image, encoding="bgr8")) 

    rospy.sleep(2)
    