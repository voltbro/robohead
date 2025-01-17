from robohead_controller_actions.main import *

import cv2
from cv_bridge import CvBridge
import numpy as np
import threading

class CircleTracker():
    def __init__(self, robohead_controller:RoboheadController, duration):
        self.is_run = True
        self.robohead_controller = robohead_controller

        self.resized_resolution = (270,270)
        self.original_resolution = (1080,1080)
        self.xy = (0,0)

        threadXY = threading.Thread(target=self.circle_xy_updater)
        threadNeck = threading.Thread(target=self.neck_mover)
        print('init circle tracker')
        threadXY.start()
        threadNeck.start()
        rospy.Timer(rospy.Duration(duration), callback=self.finish, oneshot=True)
        print('let`s go')
        # threadXY.join()
        # threadNeck.join()

    def finish(self, e):
        self.is_run = False
        print('finish!')

    def neck_mover(self):
        msg = NeckSetAngleRequest()
        msg.is_blocking = 1
        msg.duration = 0.1
        msg.vertical_angle = 0
        cur_horizontal_angle = 0
        cur_vertical_angle = 0
        step = 2
        center_x = int(self.resized_resolution[0]/2)
        center_y = int(self.resized_resolution[1]/2)
        while self.is_run:
            xy = self.xy
            # print("xy:", xy)
            if xy[0]<(center_x-15):
                if (cur_horizontal_angle+step)<=40:
                    cur_horizontal_angle+=step
                    msg.horizontal_angle = cur_horizontal_angle
            if xy[0]>(center_x+15):
                if (cur_horizontal_angle-step)>=-40:
                    cur_horizontal_angle-=step
                    msg.horizontal_angle = cur_horizontal_angle

            if xy[1]<(center_y-15):
                if (cur_vertical_angle+step)<=40:
                    cur_vertical_angle+=step
                    msg.vertical_angle = cur_vertical_angle
            if xy[1]>(center_y+15):
                if (cur_vertical_angle-step)>=-40:
                    cur_vertical_angle-=step
                    msg.vertical_angle = cur_vertical_angle
            # print("cur:", cur_horizontal_angle, cur_vertical_angle)
            # print("msg", msg.horizontal_angle, msg.vertical_angle)
            self.robohead_controller.neck_driver_srv_NeckSetAngle(msg)

    def circle_xy_updater(self):
        cvBridge = CvBridge()
        prev_img = self.robohead_controller.cv_camera_image_raw

        cv_image = cvBridge.imgmsg_to_cv2(prev_img, "bgr8")

        while self.is_run:
            if (prev_img!=self.robohead_controller.cv_camera_image_raw):
                prev_img = self.robohead_controller.cv_camera_image_raw

                cv_image = cvBridge.imgmsg_to_cv2(prev_img, "bgr8")
                cv_image = cv2.resize(cv_image[:self.original_resolution[0],:self.original_resolution[1]], self.resized_resolution)
                cv_image = cv2.flip(cv_image, 1)

                # blured_image  = cv2.medianBlur(cv_image, 15)
                blured_image = cv2.GaussianBlur(cv_image, (15, 15), 0)

                hsv = cv2.cvtColor(blured_image, cv2.COLOR_BGR2HSV)
                #fill in the values you obtained previously over here
                #https://botforge.wordpress.com/2016/07/11/object-tracking-and-following-with-opencv-python/
                #https://magicwinnie.github.io/projects/hsv_selection/hsv_selection.html
                hlow = 20
                slow = 0
                vlow = 205
                hhigh = 90
                shigh = 255
                vhigh = 255
                HSVLOW  = np.array([hlow, slow, vlow])
                HSVHIGH = np.array([hhigh, shigh, vhigh])
                mask = cv2.inRange(hsv,HSVLOW, HSVHIGH)
                # res = cv2.bitwise_and(blured_image,blured_image, mask =mask)

                edged = cv2.Canny(mask, 50, 150)
                cnts = cv2.findContours(edged, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[0]
                
                # if contours are present
                if len(cnts):
                    c = max(cnts, key=cv2.contourArea)
                    (x, y), radius = cv2.minEnclosingCircle(c)

                    if radius>10:
                        xy = (int(x), int(y))
                        self.xy = xy
                        cv2.circle(cv_image, xy, int(radius), (0, 255, 0), 2)
                        cv2.circle(cv_image, xy, 1, (0, 0, 255), 3)
                cv2.rectangle(cv_image, (270//2-15,270//2-15), (270//2+15,270//2+15), (0,0,255), 1)
                cv_image = cv2.resize(cv_image, self.original_resolution)
                self.robohead_controller.display_driver_pub_PlayMedia.publish(cvBridge.cv2_to_imgmsg(cv_image, encoding="bgr8"))


def run(robohead_controller:RoboheadController, cmds:str): # Обязательно наличие этой функции, именно она вызывается при голосовой команде
    script_path = os.path.dirname(os.path.abspath(__file__)) + '/'

    msg = EarsSetAngleRequest()
    msg.left_ear_angle = -30
    msg.right_ear_angle = -30
    robohead_controller.ears_driver_srv_EarsSetAngle(msg)

    obj = CircleTracker(robohead_controller, 25)
    while obj.is_run:
        pass

    msg = PlayAudioRequest()
    msg.path_to_file = script_path + 'make_photo.mp3'
    msg.is_blocking = 1
    msg.is_cycled = 0
    robohead_controller.speakers_driver_srv_PlayAudio(msg)