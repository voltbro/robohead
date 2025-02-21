from robohead_controller_actions.main import *

import cv2
from cv_bridge import CvBridge
import numpy as np
import threading

class BallTracker():
    def __init__(self, robohead_controller:RoboheadController):
        self.is_run = True
        self.robohead_controller = robohead_controller

        self.resized_camera_resolution = (270,270)
        self.original_camera_resolution = (640,480)
        self.screen_resolution = (1080,1080)
        self.ball_xy = (0,0)

        self.thread_ball_tracker = threading.Thread(target=self.ball_tracker)
        self.thread_neck_mover = threading.Thread(target=self.neck_mover)

        self.neck_mover_step_duration = 0.1
        self.neck_mover_step_value = 2
        self.neck_mover_constraint_vertical = (-30,30)
        self.neck_mover_constraint_horizontal = (-30,30)

        self.delta_k = 5/((self.resized_camera_resolution[0]//2)**2+(self.resized_camera_resolution[1]//2)**2)**0.5

        self.threshold_zone = 15
        self.hsv_filter = ((16,40),(180,255),(120,255)) # (hlow,hhigh), (slow,shigh), (vlow,vhigh)
        print('init circle tracker')
        print('let`s go')
        # threadXY.join()
        # threadNeck.join()

    def start(self, duration):
        self.thread_ball_tracker.start()
        self.thread_neck_mover.start()
        rospy.Timer(rospy.Duration(duration), callback=self.finish, oneshot=True)

    def join(self):
        self.thread_ball_tracker.join()
        self.thread_neck_mover.join()

    def finish(self, e):
        self.is_run = False
        print('finish!')

    def neck_mover(self):
        msg = NeckSetAngleRequest()
        msg.is_blocking = 1
        msg.duration = self.neck_mover_step_duration
        msg.vertical_angle = 0
        msg.horizontal_angle = 0

        cur_horizontal_angle = 0
        cur_vertical_angle = 0

        step = self.neck_mover_step_value
        threshold_zone = self.threshold_zone

        center_x = int(self.resized_camera_resolution[0]/2)
        center_y = int(self.resized_camera_resolution[1]/2)
        prev_ball_xy = self.ball_xy
        while self.is_run and not rospy.is_shutdown():
            x, y = self.ball_xy
            if (prev_ball_xy[0]==x) and (prev_ball_xy[1]==y):
                continue
            delta = (((center_x-x)**2+(center_y-y)**2)**0.5)*self.delta_k
            delta = min(max(delta, 1),3)
            step_d = int(step*delta)
            # print(delta)

            if x<(center_x-threshold_zone):
                if (cur_horizontal_angle+step_d)<=self.neck_mover_constraint_horizontal[1]:
                    cur_horizontal_angle+=step_d
                    msg.horizontal_angle = cur_horizontal_angle
            elif x>(center_x+threshold_zone):
                if (cur_horizontal_angle-step_d)>=self.neck_mover_constraint_horizontal[0]:
                    cur_horizontal_angle-=step_d
                    msg.horizontal_angle = cur_horizontal_angle

            if y<(center_y-threshold_zone):
                if (cur_vertical_angle+step_d)<=self.neck_mover_constraint_vertical[1]:
                    cur_vertical_angle+=step_d
                    msg.vertical_angle = cur_vertical_angle
            elif y>(center_y+threshold_zone):
                if (cur_vertical_angle-step_d)>=self.neck_mover_constraint_vertical[0]:
                    cur_vertical_angle-=step_d
                    msg.vertical_angle = cur_vertical_angle
            # print('cur: ',cur_vertical_angle, cur_horizontal_angle)
            self.robohead_controller.neck_driver_srv_NeckSetAngle(msg)

    def ball_tracker(self):
        cvBridge = CvBridge()
        prev_img = self.robohead_controller.cv_camera_image_raw

        cv_image = cvBridge.imgmsg_to_cv2(prev_img, "bgr8")
        script_path = os.path.dirname(os.path.abspath(__file__)) + '/'

        while self.is_run and not rospy.is_shutdown():
            if (prev_img!=self.robohead_controller.cv_camera_image_raw):
                prev_img = self.robohead_controller.cv_camera_image_raw

                cv_image = cvBridge.imgmsg_to_cv2(prev_img, "bgr8")
                # cv2.imwrite(script_path+'1_orig.png', cv_image)

                rect = min(self.original_camera_resolution)
                cv_image = cv2.resize(cv_image[:rect,:rect], self.resized_camera_resolution)
                cv_image = cv2.flip(cv_image, 1)
                # cv2.imwrite(script_path+'2_resized.png', cv_image)
                

                blured_image = cv2.GaussianBlur(cv_image, (15, 15), 0)
                # cv2.imwrite(script_path+'3_blure.png', blured_image)

                hsv = cv2.cvtColor(blured_image, cv2.COLOR_BGR2HSV)
                # cv2.imwrite(script_path+'4_hsv.png', hsv)
                #fill in the values you obtained previously over here
                #https://botforge.wordpress.com/2016/07/11/object-tracking-and-following-with-opencv-python/
                #https://magicwinnie.github.io/projects/hsv_selection/hsv_selection.html
                hlow = self.hsv_filter[0][0]
                slow = self.hsv_filter[1][0]
                vlow = self.hsv_filter[2][0]
                hhigh = self.hsv_filter[0][1]
                shigh = self.hsv_filter[1][1]
                vhigh = self.hsv_filter[2][1]
                HSVLOW  = np.array([hlow, slow, vlow])
                HSVHIGH = np.array([hhigh, shigh, vhigh])
                mask = cv2.inRange(hsv,HSVLOW, HSVHIGH)
                # cv2.imwrite(script_path+'5_hsv_filtered.png', mask)

                edged = cv2.Canny(mask, 50, 150)
                # cv2.imwrite(script_path+'6_edged.png', edged)
                cnts = cv2.findContours(edged, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[0]
                
                if len(cnts):
                    c = max(cnts, key=cv2.contourArea)
                    (x, y), radius = cv2.minEnclosingCircle(c)

                    if radius>10:
                        xy = (int(x), int(y))
                        self.ball_xy = xy
                        cv2.circle(cv_image, xy, int(radius), (0, 255, 0), 2)
                        cv2.circle(cv_image, xy, 1, (0, 0, 255), 3)
                lu_angle = (self.resized_camera_resolution[0]//2-self.threshold_zone, self.resized_camera_resolution[1]//2-self.threshold_zone)
                rd_angle = (self.resized_camera_resolution[0]//2+self.threshold_zone, self.resized_camera_resolution[1]//2+self.threshold_zone)
                cv2.rectangle(cv_image, lu_angle, rd_angle, (0,0,255), 1)
                cv_image = cv2.resize(cv_image, self.screen_resolution)
                self.robohead_controller.display_driver_pub_PlayMedia.publish(cvBridge.cv2_to_imgmsg(cv_image, encoding="bgr8"))


def run(robohead_controller:RoboheadController, cmds:str): # Обязательно наличие этой функции, именно она вызывается при голосовой команде
    script_path = os.path.dirname(os.path.abspath(__file__)) + '/'

    msg = EarsSetAngleRequest()
    msg.left_ear_angle = -30
    msg.right_ear_angle = -30
    robohead_controller.ears_driver_srv_EarsSetAngle(msg)

    obj = BallTracker(robohead_controller)
    obj.start(20)
    obj.join()

    msg = PlayAudioRequest()
    msg.path_to_file = script_path + 'make_photo.mp3'
    msg.is_blocking = 1
    msg.is_cycled = 0
    robohead_controller.speakers_driver_srv_PlayAudio(msg)