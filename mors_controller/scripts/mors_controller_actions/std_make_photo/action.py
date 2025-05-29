from mors_controller_actions.main import *

import cv2
from cv_bridge import CvBridge
import numpy as np

def run(mors_controller:MorsController, cmds:str): # Обязательно наличие этой функции, именно она вызывается при голосовой команде
    script_path = os.path.dirname(os.path.abspath(__file__)) + '/'

    msg = EarsSetAngleRequest()
    msg.left_ear_angle = -30
    msg.right_ear_angle = -30
    mors_controller.ears_driver_srv_EarsSetAngle(msg)

    msg = NeckSetAngleRequest()
    msg.horizontal_angle = 0
    msg.vertical_angle = 30
    msg.duration = 1
    msg.is_blocking = 1
    mors_controller.neck_driver_srv_NeckSetAngle(msg)

    cvBridge = CvBridge()

    font = cv2.FONT_HERSHEY_SIMPLEX
    fontScale = 10
    fontColor = (255,255,255)
    thickness = 20
    lineType = 5
    
    prev_img = mors_controller.usb_cam_image_raw

    for num in range(3,0,-1):
        str_num = str(num)
        timer_start = rospy.get_time()
        while (rospy.get_time()-timer_start)<1:
            if (prev_img!=mors_controller.usb_cam_image_raw):
                prev_img = mors_controller.usb_cam_image_raw

                cv_image = cvBridge.imgmsg_to_cv2(prev_img, "bgr8")
                cv_image = cv2.resize(cv_image, (1080, 1080))

                bottomLeftCornerOfText = (1080//2-cv2.getTextSize(str_num, font, fontScale, thickness)[0][0]//2, 1080//2+cv2.getTextSize(str_num, font, fontScale, thickness)[0][1]//2)
                cv2.putText(cv_image, str_num, tuple(bottomLeftCornerOfText), 
                font, fontScale, fontColor, thickness, lineType)

                mors_controller.display_driver_pub_PlayMedia.publish(cvBridge.cv2_to_imgmsg(cv_image, encoding="bgr8")) 

    msg = PlayAudioRequest()
    msg.path_to_file = script_path + 'make_photo.mp3'
    msg.is_blocking = 0
    msg.is_cycled = 0
    mors_controller.speakers_driver_srv_PlayAudio(msg)

    rospy.sleep(0.5)
    cvBridge = CvBridge()
    cv_image = cvBridge.imgmsg_to_cv2(mors_controller.usb_cam_image_raw, "bgr8")
    cv_image = cv2.resize(cv_image, (1080, 1080))
    mors_controller.display_driver_pub_PlayMedia.publish(cvBridge.cv2_to_imgmsg(cv_image, encoding="bgr8")) 

    rospy.sleep(4)