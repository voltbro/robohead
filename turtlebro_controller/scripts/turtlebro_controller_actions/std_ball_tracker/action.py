#https://magicwinnie.github.io/projects/hsv_selection/hsv_selection.html
from turtlebro_controller_actions.main import *

import cv2
from cv_bridge import CvBridge
import numpy as np
import threading
import math

class BallTracker():
    def __init__(self, turtlebro_controller:TurtlebroController, script_path):
        self.is_run = True
        self.turtlebro_controller = turtlebro_controller
        self.cvBridge = CvBridge()
        self.script_path = script_path

        self.calibration_radius = 40 # радиус, который срезается для извлечения калибровочного цвета
        self.calibration_radius_delta = 10 # погрешность, которая отнимается от радиуса срезаемого изображения (получается срез = круг радиусом radius_cal-delta_cal), чтобы человек наверняка попал

        self.tracker_zone_radius = 30
        self.tracker_distance_radius = 40
        self.tracker_distance_radius_delta = 5

        self.hsv_filter = ((16,40),(180,255),(120,255)) # (hlow,hhigh), (slow,shigh), (vlow,vhigh)
        # hsv_filter изменяется при калибровке

        self.resized_camera_resolution = (270,270)
        self.original_camera_resolution = (640,480)
        self.screen_resolution = (1080,1080)
        self.ball_xyr = (int(self.resized_camera_resolution[0]//2), int(self.resized_camera_resolution[1]//2), self.tracker_distance_radius)

        self.thread_ball_tracker = threading.Thread(target=self.ball_tracker)
        self.thread_mover = threading.Thread(target=self.mover)

        self.mover_neck_step_duration = 0.1
        self.mover_neck_step_value = 2
        self.mover_constraint_vertical = (-30,30)
        self.mover_constraint_horizontal = (-30,30)

    def start(self, duration_tracking, duration_calibrate):
        self.hsv_filter = self.calibrate(duration_calibrate)
        self.thread_ball_tracker.start()
        self.thread_mover.start()
        rospy.Timer(rospy.Duration(duration_tracking), callback=self.finish, oneshot=True)

    def join(self):
        self.thread_ball_tracker.join()
        self.thread_mover.join()

    def finish(self, e):
        self.is_run = False
    def sign(self, x:int):
        if x==0: return 0
        elif x<0: return -1
        elif x>0: return 1
    def mover(self):
        msg_neck = NeckSetAngleRequest()
        msg_neck.is_blocking = 0
        msg_neck.duration = self.mover_neck_step_duration
        msg_neck.vertical_angle = 0
        
        mover_neck_step_value = self.mover_neck_step_value
        cur_vertical_angle = 0
        cur_horizontal_angle = 0

        msg_cmd_vel = Twist()

        resized_camera_resolution = self.resized_camera_resolution
        center_x = int(resized_camera_resolution[0]/2)
        center_y = int(resized_camera_resolution[1]/2)

        tracker_distance_radius = self.tracker_distance_radius
        tracker_distance_radius_delta = self.tracker_distance_radius_delta
        tracker_zone_radius = self.tracker_zone_radius
        prev_ball_xyr = self.ball_xyr

        timer_start = rospy.get_time()
        is_rotating = False

        while self.is_run and not rospy.is_shutdown():
            x, y, r = self.ball_xyr

            if (rospy.get_time() - timer_start) > 2:
                msg_cmd_vel = Twist()
                self.turtlebro_controller.turtlebro_pub_cmd_vel.publish(msg_cmd_vel)
                timer_start = rospy.get_time()

            if (prev_ball_xyr[0]==x) and (prev_ball_xyr[1]==y) and (prev_ball_xyr[2]==r):
                continue
            timer_start = rospy.get_time()

            prev_ball_xyr = (x,y,r)

            delta_r = tracker_distance_radius - r
            delta_x = x - center_x
            delta_y = center_y - y

            # слежение за шариком по расстоянию
            # if abs(delta_r) > tracker_distance_radius_delta:
            #     velocity = min(max(0.1*abs(delta_r)/(resized_camera_resolution[0]/4), 0.05), 0.2)
            #     msg_cmd_vel.linear.x = velocity*self.sign(delta_r)
            # else:
            #     msg_cmd_vel.linear.x = 0


            if (delta_x**2+delta_y**2) > tracker_zone_radius**2:
                if (abs(cur_vertical_angle+mover_neck_step_value*self.sign(delta_y))<=30):
                    cur_vertical_angle+=mover_neck_step_value*self.sign(delta_y)
                    msg_neck.vertical_angle = int(cur_vertical_angle)

                if not is_rotating and (abs(delta_x) <= tracker_zone_radius):
                    msg_cmd_vel.angular.z = 0
                elif is_rotating:
                    velocity = min(max(0.2*abs(delta_x)/(resized_camera_resolution[0]/4), 0.1), 0.4)
                    msg_cmd_vel.angular.z = velocity*self.sign(delta_x)

                    if (abs(cur_horizontal_angle+mover_neck_step_value*self.sign(delta_x))<=30):
                        cur_horizontal_angle+=mover_neck_step_value*self.sign(delta_x)
                        msg_neck.horizontal_angle = int(cur_horizontal_angle)
                    
                    if abs(cur_horizontal_angle)<=5:
                        is_rotating = False
                else:
                    if (abs(cur_horizontal_angle-mover_neck_step_value*self.sign(delta_x))<=30):
                        cur_horizontal_angle-=mover_neck_step_value*self.sign(delta_x)
                        msg_neck.horizontal_angle = int(cur_horizontal_angle)
                    else:
                        is_rotating = True
            else:
                msg_cmd_vel.angular.z = 0

            self.turtlebro_controller.neck_driver_srv_NeckSetAngle(msg_neck)
            self.turtlebro_controller.turtlebro_pub_cmd_vel.publish(msg_cmd_vel)

        msg_cmd_vel = Twist()
        self.turtlebro_controller.turtlebro_pub_cmd_vel.publish(msg_cmd_vel)

    def ball_tracker(self):
        prev_img = self.turtlebro_controller.usb_cam_image_raw
        resized_camera_resolution = self.resized_camera_resolution
        original_camera_resolution = self.original_camera_resolution
        screen_resolution = self.screen_resolution
        # центр resized изображения
        center_x_resized = resized_camera_resolution[0]//2
        center_y_resized = resized_camera_resolution[1]//2

        tracker_zone_radius = self.tracker_zone_radius
        tracker_distance_radius = self.tracker_distance_radius
        tracker_distance_radius_delta = self.tracker_distance_radius_delta

        hlow = self.hsv_filter[0][0]
        slow = self.hsv_filter[1][0]
        vlow = self.hsv_filter[2][0]
        hhigh = self.hsv_filter[0][1]
        shigh = self.hsv_filter[1][1]
        vhigh = self.hsv_filter[2][1]
        HSVLOW  = np.array([hlow, slow, vlow])
        HSVHIGH = np.array([hhigh, shigh, vhigh])
        rect_cut = min(original_camera_resolution)

        while self.is_run and not rospy.is_shutdown():
            if (prev_img!=self.turtlebro_controller.usb_cam_image_raw):
                prev_img = self.turtlebro_controller.usb_cam_image_raw

                cv_image = self.cvBridge.imgmsg_to_cv2(prev_img, "bgr8")
                # cv2.imwrite(self.script_path+'1_orig.png', cv_image)
                
                cv_image = cv2.resize(cv_image[:rect_cut,:rect_cut], resized_camera_resolution)
                cv_image = cv2.flip(cv_image, 1)
                # cv2.imwrite(self.script_path+'2_resized.png', cv_image)

                blured_image = cv2.GaussianBlur(cv_image, (15, 15), 0)
                # cv2.imwrite(self.script_path+'3_blure.png', blured_image)

                hsv = cv2.cvtColor(blured_image, cv2.COLOR_BGR2HSV)
                # cv2.imwrite(self.script_path+'4_hsv_format.png', hsv)

                mask = cv2.inRange(hsv,HSVLOW, HSVHIGH)
                # cv2.imwrite(self.script_path+'5_hsv_filtered.png', mask)
                kernel = np.ones((3, 3), np.uint8)
                dilation = cv2.dilate(mask, kernel, iterations=1) 
                # cv2.imwrite(self.script_path+'51_dilated.png', dilation)

                edged = cv2.Canny(dilation, 50, 150)
                # cv2.imwrite(self.script_path+'6_edged.png', edged)
                cnts = cv2.findContours(edged, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[0]
                
                if len(cnts):
                    c = max(cnts, key=cv2.contourArea)
                    (x, y), radius = cv2.minEnclosingCircle(c)

                    if (radius>10) and (radius<100):
                        xy = (int(x), int(y))
                        self.ball_xyr = (int(x), int(y), int(radius))

                        cv2.circle(cv_image, xy, int(radius), (0, 255, 0), 2)
                        cv2.circle(cv_image, xy, 1, (0, 0, 255), 3)
                        cv2.circle(cv_image, xy, int(tracker_distance_radius+tracker_distance_radius_delta), (255, 0, 0), 1)
                        cv2.circle(cv_image, xy, int(tracker_distance_radius-tracker_distance_radius_delta), (255, 0, 0), 1)

                cv2.circle(cv_image, (center_x_resized,center_y_resized), int(tracker_zone_radius), (0, 0, 255), 1)
                cv_image = cv2.resize(cv_image, screen_resolution)
                self.turtlebro_controller.display_driver_pub_PlayMedia.publish(self.cvBridge.cv2_to_imgmsg(cv_image, encoding="bgr8"))

    def calibrate(self, duration_calibrate):
        prev_img = self.turtlebro_controller.usb_cam_image_raw

        resized_camera_resolution = self.resized_camera_resolution
        # центр resized изображения
        center_x_resized = resized_camera_resolution[0]//2
        center_y_resized = resized_camera_resolution[1]//2

        # параметры для текста (обратный таймер с циферками)
        font = cv2.FONT_HERSHEY_SIMPLEX
        fontScale = 10
        fontColor = (255,0,0)
        thickness = 20
        lineType = 5

        calibration_radius = self.calibration_radius
        calibration_radius_delta = self.calibration_radius_delta
        
        rect_cut = min(self.original_camera_resolution)

        timer_start = rospy.get_time()
        while ((rospy.get_time()-timer_start)<duration_calibrate) and not rospy.is_shutdown():
            if (prev_img!=self.turtlebro_controller.usb_cam_image_raw):
                prev_img = self.turtlebro_controller.usb_cam_image_raw

                str_num = str(duration_calibrate-int(rospy.get_time()-timer_start))
                cv_image = self.cvBridge.imgmsg_to_cv2(prev_img, "bgr8")

                
                cv_image = cv2.resize(cv_image[:rect_cut,:rect_cut], resized_camera_resolution)
                cv_image = cv2.flip(cv_image, 1)
                
                cv2.circle(cv_image, (center_x_resized, center_y_resized), calibration_radius, (0, 255, 0), 2)
                cv2.circle(cv_image, (center_x_resized, center_y_resized), calibration_radius-calibration_radius_delta, (255, 0, 0), 1)

                screen_img = cv2.resize(cv_image, self.screen_resolution)
                bottomLeftCornerOfText = (self.screen_resolution[0]//2-cv2.getTextSize(str_num, font, fontScale, thickness)[0][0]//2, self.screen_resolution[1]//2-2*cv2.getTextSize(str_num, font, fontScale, thickness)[0][1]//2-calibration_radius)
                cv2.putText(screen_img, str_num, tuple(bottomLeftCornerOfText), 
                font, fontScale, fontColor, thickness, lineType)
                self.turtlebro_controller.display_driver_pub_PlayMedia.publish(self.cvBridge.cv2_to_imgmsg(screen_img, encoding="bgr8"))
        
        cv_image = self.cvBridge.imgmsg_to_cv2(prev_img, "bgr8")
        cv_image = cv2.resize(cv_image[:rect_cut,:rect_cut], resized_camera_resolution)
        cv_image = cv2.flip(cv_image, 1)

        rect = cv_image[int(center_x_resized-calibration_radius+calibration_radius_delta):int(center_x_resized+calibration_radius-calibration_radius_delta), int(center_y_resized-calibration_radius+calibration_radius_delta):int(center_y_resized+calibration_radius-calibration_radius_delta)]
        mask = np.zeros(rect.shape[:2], dtype="uint8")
        cv2.circle(mask, (int(rect.shape[0]//2), int(rect.shape[1]//2)), calibration_radius-calibration_radius_delta, 255, -1)
        masked = cv2.bitwise_and(rect, rect, mask=mask)

        hsv = self.get_hsv_range(masked)
        print("hsv_filter:", hsv)
        return hsv
    
    def get_hsv_range(self, img):
        # пробегаем по изображеняи img и выделяем диапазон цветов на нем

        # cv2.imwrite(self.script_path+'get_hsv_range_img.png', img)
        blured_image = cv2.GaussianBlur(img, (15, 15), 0)
        hsv_image = cv2.cvtColor(blured_image, cv2.COLOR_BGR2HSV)
        rows, cols, _ = hsv_image.shape

        hsv_filter = [[180,0],[255,0],[255,0]] # (hlow,hhigh), (slow,shigh), (vlow,vhigh)
        for row in range(rows):
            for col in range(cols):
                if (hsv_image[row][col][1]<=70) or (hsv_image[row][col][2]<=70):
                    continue
                hsv_filter[0][0] = min(hsv_image[row][col][0], hsv_filter[0][0])
                hsv_filter[0][1] = max(hsv_image[row][col][0], hsv_filter[0][1])

                hsv_filter[1][0] = min(hsv_image[row][col][1], hsv_filter[1][0])
                hsv_filter[1][1] = max(hsv_image[row][col][1], hsv_filter[1][1])

                hsv_filter[2][0] = min(hsv_image[row][col][2], hsv_filter[2][0])
                hsv_filter[2][1] = max(hsv_image[row][col][2], hsv_filter[2][1])
        return hsv_filter



def run(turtlebro_controller:TurtlebroController, cmds:str): # Обязательно наличие этой функции, именно она вызывается при голосовой команде
    script_path = os.path.dirname(os.path.abspath(__file__)) + '/'

    msg = EarsSetAngleRequest()
    msg.left_ear_angle = -30
    msg.right_ear_angle = -30
    turtlebro_controller.ears_driver_srv_EarsSetAngle(msg)

    msg = PlayAudioRequest()
    msg.path_to_file = script_path + 'calibrate_voice.mp3'
    msg.is_blocking = 0
    msg.is_cycled = 0
    turtlebro_controller.speakers_driver_srv_PlayAudio(msg)

    obj = BallTracker(turtlebro_controller, script_path)
    obj.start(35, 7)
    obj.join()

    msg = PlayAudioRequest()
    msg.path_to_file = script_path + 'finish_voice.mp3'
    msg.is_blocking = 1
    msg.is_cycled = 0
    turtlebro_controller.speakers_driver_srv_PlayAudio(msg)