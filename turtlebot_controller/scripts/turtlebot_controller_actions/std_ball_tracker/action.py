#https://magicwinnie.github.io/projects/hsv_selection/hsv_selection.html
from turtlebot_controller_actions.main import *

import cv2
from cv_bridge import CvBridge
import numpy as np
import threading

class BallTracker():
    def __init__(self, turtlebot_controller:TurtlebotController, script_path):
        self.is_run = True
        self.turtlebot_controller = turtlebot_controller
        self.cvBridge = CvBridge()
        self.script_path = script_path

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

        self.delta_k = 3/((self.resized_camera_resolution[0]//2)**2+(self.resized_camera_resolution[1]//2)**2)**0.5
        # показывает, во сколько раз надо увеличить neck_mover_step_value, если шарик находится в самом краю изображения

        self.threshold_zone = 15
        self.hsv_filter = ((16,40),(180,255),(120,255)) # (hlow,hhigh), (slow,shigh), (vlow,vhigh)
        # hsv_filter изменяется при калибровке

        self.radius_cal = 30 # радиус, который срезается для извлечения калибровочного цвета
        self.delta_cal = 10 # погрешность, которая прибавляется к радиусу срезаемого изображения (круг радиусом radius_cal), чтобы человек наверняка попал

    def start(self, duration_tracking, duration_calibrate):
        self.hsv_filter = self.calibrate(duration_calibrate, self.radius_cal, self.delta_cal)
        self.thread_ball_tracker.start()
        self.thread_neck_mover.start()
        rospy.Timer(rospy.Duration(duration_tracking), callback=self.finish, oneshot=True)

    def join(self):
        self.thread_ball_tracker.join()
        self.thread_neck_mover.join()

    def finish(self, e):
        self.is_run = False

    def neck_mover(self):
        msg = NeckSetAngleRequest()
        msg.is_blocking = 1
        msg.duration = self.neck_mover_step_duration
        msg.vertical_angle = 0
        msg.horizontal_angle = 0

        msg_twist = Twist()

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

            if x<(center_x-threshold_zone):
                msg_twist.angular.z = -0.2*delta
                # if (cur_horizontal_angle+step_d)<=self.neck_mover_constraint_horizontal[1]:
                #     cur_horizontal_angle+=step_d
                #     msg.horizontal_angle = cur_horizontal_angle
            elif x>(center_x+threshold_zone):
                msg_twist.angular.z = 0.2*delta
                # if (cur_horizontal_angle-step_d)>=self.neck_mover_constraint_horizontal[0]:
                #     cur_horizontal_angle-=step_d
                #     msg.horizontal_angle = cur_horizontal_angle
            else:
                msg_twist.angular.z = 0

            if y<(center_y-threshold_zone):
                if (cur_vertical_angle+step_d)<=self.neck_mover_constraint_vertical[1]:
                    cur_vertical_angle+=step_d
                    msg.vertical_angle = cur_vertical_angle
            elif y>(center_y+threshold_zone):
                if (cur_vertical_angle-step_d)>=self.neck_mover_constraint_vertical[0]:
                    cur_vertical_angle-=step_d
                    msg.vertical_angle = cur_vertical_angle

            self.turtlebot_controller.neck_driver_srv_NeckSetAngle(msg)
            self.turtlebot_controller.turtlebot_pub_cmd_vel.publish(msg_twist)
        msg_twist.angular.z = 0
        self.turtlebot_controller.turtlebot_pub_cmd_vel.publish(msg_twist)

    def ball_tracker(self):
        prev_img = self.turtlebot_controller.usb_cam_image_raw

        while self.is_run and not rospy.is_shutdown():
            if (prev_img!=self.turtlebot_controller.usb_cam_image_raw):
                prev_img = self.turtlebot_controller.usb_cam_image_raw

                cv_image = self.cvBridge.imgmsg_to_cv2(prev_img, "bgr8")
                # cv2.imwrite(self.script_path+'1_orig.png', cv_image)

                rect = min(self.original_camera_resolution)
                cv_image = cv2.resize(cv_image[:rect,:rect], self.resized_camera_resolution)
                cv_image = cv2.flip(cv_image, 1)
                # cv2.imwrite(self.script_path+'2_resized.png', cv_image)

                blured_image = cv2.GaussianBlur(cv_image, (15, 15), 0)
                # cv2.imwrite(self.script_path+'3_blure.png', blured_image)

                hsv = cv2.cvtColor(blured_image, cv2.COLOR_BGR2HSV)
                # cv2.imwrite(self.script_path+'4_hsv.png', hsv)

                hlow = self.hsv_filter[0][0]
                slow = self.hsv_filter[1][0]
                vlow = self.hsv_filter[2][0]
                hhigh = self.hsv_filter[0][1]
                shigh = self.hsv_filter[1][1]
                vhigh = self.hsv_filter[2][1]
                HSVLOW  = np.array([hlow, slow, vlow])
                HSVHIGH = np.array([hhigh, shigh, vhigh])
                mask = cv2.inRange(hsv,HSVLOW, HSVHIGH)
                # cv2.imwrite(self.script_path+'5_hsv_filtered.png', mask)

                edged = cv2.Canny(mask, 50, 150)
                # cv2.imwrite(self.script_path+'6_edged.png', edged)
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
                self.turtlebot_controller.display_driver_pub_PlayMedia.publish(self.cvBridge.cv2_to_imgmsg(cv_image, encoding="bgr8"))

    def calibrate(self, duration_calibrate, radius_cal, delta_cal):
        prev_img = self.turtlebot_controller.usb_cam_image_raw

        # центр resized изображения
        center_x_resized = self.resized_camera_resolution[0]//2
        center_y_resized = self.resized_camera_resolution[1]//2

        # параметры для текста (обратный таймер с циферками)
        font = cv2.FONT_HERSHEY_SIMPLEX
        fontScale = 10
        fontColor = (255,0,0)
        thickness = 20
        lineType = 5

        timer_start = rospy.get_time()
        while ((rospy.get_time()-timer_start)<duration_calibrate) and not rospy.is_shutdown():
            if (prev_img!=self.turtlebot_controller.usb_cam_image_raw):
                prev_img = self.turtlebot_controller.usb_cam_image_raw

                str_num = str(duration_calibrate-int(rospy.get_time()-timer_start))
                cv_image = self.cvBridge.imgmsg_to_cv2(prev_img, "bgr8")

                rect = min(self.original_camera_resolution)
                cv_image = cv2.resize(cv_image[:rect,:rect], self.resized_camera_resolution)
                cv_image = cv2.flip(cv_image, 1)
                rect = cv_image.copy()
                
                cv2.circle(cv_image, (center_x_resized, center_y_resized), radius_cal+delta_cal, (0, 255, 0), 2)
                cv2.circle(cv_image, (center_x_resized, center_y_resized), radius_cal, (255, 0, 0), 1)

                screen_img = cv2.resize(cv_image, self.screen_resolution)
                bottomLeftCornerOfText = (self.screen_resolution[0]//2-cv2.getTextSize(str_num, font, fontScale, thickness)[0][0]//2, self.screen_resolution[1]//2-2*cv2.getTextSize(str_num, font, fontScale, thickness)[0][1]//2-radius_cal)
                cv2.putText(screen_img, str_num, tuple(bottomLeftCornerOfText), 
                font, fontScale, fontColor, thickness, lineType)
                self.turtlebot_controller.display_driver_pub_PlayMedia.publish(self.cvBridge.cv2_to_imgmsg(screen_img, encoding="bgr8"))
        
        rect = rect[int(center_x_resized-radius_cal):int(center_x_resized+radius_cal), int(center_y_resized-radius_cal):int(center_y_resized+radius_cal)]
        mask = np.zeros(rect.shape[:2], dtype="uint8")
        cv2.circle(mask, (int(rect.shape[0]//2), int(rect.shape[1]//2)), radius_cal, 255, -1)
        masked = cv2.bitwise_and(rect, rect, mask=mask)

        hsv = self.get_hsv_range(masked)
        # print("hsv_filter:", hsv)
        return hsv
    
    def get_hsv_range(self, img):
        # пробегаем по изображеняи img и выделяем диапазон цветов на нем

        blured_image = cv2.GaussianBlur(img, (15, 15), 0)
        hsv_image = cv2.cvtColor(blured_image, cv2.COLOR_BGR2HSV)
        rows, cols, _ = hsv_image.shape

        hsv_filter = [[180,0],[255,0],[255,0]] # (hlow,hhigh), (slow,shigh), (vlow,vhigh)
        for row in range(rows):
            for col in range(cols):
                if (hsv_image[row][col][1]<=100) or (hsv_image[row][col][2]<=100):
                    continue
                hsv_filter[0][0] = min(hsv_image[row][col][0], hsv_filter[0][0])
                hsv_filter[0][1] = max(hsv_image[row][col][0], hsv_filter[0][1])

                hsv_filter[1][0] = min(hsv_image[row][col][1], hsv_filter[1][0])
                hsv_filter[1][1] = max(hsv_image[row][col][1], hsv_filter[1][1])

                hsv_filter[2][0] = min(hsv_image[row][col][2], hsv_filter[2][0])
                hsv_filter[2][1] = max(hsv_image[row][col][2], hsv_filter[2][1])
        return hsv_filter



def run(turtlebot_controller:TurtlebotController, cmds:str): # Обязательно наличие этой функции, именно она вызывается при голосовой команде
    script_path = os.path.dirname(os.path.abspath(__file__)) + '/'

    msg = EarsSetAngleRequest()
    msg.left_ear_angle = -30
    msg.right_ear_angle = -30
    turtlebot_controller.ears_driver_srv_EarsSetAngle(msg)

    msg = PlayAudioRequest()
    msg.path_to_file = script_path + 'calibrate_voice.mp3'
    msg.is_blocking = 0
    msg.is_cycled = 0
    turtlebot_controller.speakers_driver_srv_PlayAudio(msg)

    obj = BallTracker(turtlebot_controller, script_path)
    obj.start(30, 10)
    obj.join()

    msg = PlayAudioRequest()
    msg.path_to_file = script_path + 'finish_voice.mp3'
    msg.is_blocking = 1
    msg.is_cycled = 0
    turtlebot_controller.speakers_driver_srv_PlayAudio(msg)