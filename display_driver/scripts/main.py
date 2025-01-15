import rospy
from display_driver.srv import PlayMedia, PlayMediaRequest, PlayMediaResponse
from geometry_msgs.msg import Pose2D

import os
import cv2
import numpy as np
import multiprocessing
import threading

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from ctypes import c_char
import signal

import evdev, select
from math import sin, cos, radians

class DisplayDriver():

    def __init__(self) -> None:
        self._height = rospy.get_param("~screen_resolution_h", 1080)
        self._width = rospy.get_param("~screen_resolution_w", 1080)
        self._color_channels = rospy.get_param("~color_channels", 3)
        self._framerate = rospy.get_param("~framerate", 60)
        self._fb_name = rospy.get_param("~fb_name", "/dev/fb0")
        self._screen_rotate = rospy.get_param("~screen_rotate", 0)

        self._use_touch = rospy.get_param("~use_touch", True)
        touchscreen_name = rospy.get_param("~touchscreen_name", "waveshare")
        self._touchscreen_resolution_h = rospy.get_param("~touchscreen_resolution_h", 4096)
        self._touchscreen_resolution_w = rospy.get_param("~touchscreen_resolution_w", 4096)
        self._touchscreen_rotate = rospy.get_param("~touch_rotate", 0)

        service_PlayMedia_name = rospy.get_param("~service_PlayMedia_name", "~PlayMedia")
        topic_PlayMedia_name = rospy.get_param("~topic_PlayMedia_name", "~PlayMedia")
        topic_touchsreen_name = rospy.get_param("~topic_touchscreen_name", "~touchscreen")

        self._semaphore = multiprocessing.Semaphore(1)
        self._showing_file = multiprocessing.Array(c_char, 4096)
        self._showing_file.value = ''.encode()
        self._is_played = multiprocessing.Value('i', 0)
        self._is_cycled = multiprocessing.Value('i', 0)
        
        self._fb = np.memmap(self._fb_name, dtype='uint8', mode='w+', shape=(self._height, self._width, self._color_channels))
        
        self._cvBridge = CvBridge()

        self._proccess_player = multiprocessing.Process(target=self._player, args=(), daemon=True)
        self._proccess_player.start()

        PlayMedia_subscriber = rospy.Subscriber(topic_PlayMedia_name, Image, callback=self._img_sub)
        PlayMedia_service = rospy.Service(service_PlayMedia_name, PlayMedia, self._requester)
        
        if self._use_touch:
            self._touchsreen_publisher = rospy.Publisher(topic_touchsreen_name, Pose2D, queue_size=1)
            flag = 0
            for device in evdev.list_devices("/dev/input/"):
                self._touch = evdev.InputDevice(device)
                if (touchscreen_name in str(self._touch.name).lower()):
                    flag = 1
                    break
            if flag==0:
                rospy.logerr(f"Touchscreen \"{touchscreen_name}\" not founded")
            self._thread_touchscreen = threading.Thread(target=self._touchscreen, args=(), daemon=True)
            self._thread_touchscreen.start()

        signal.signal(signal.SIGINT, self.stop)
        rospy.loginfo("display_driver INITED")

    def _img_sub(self, image_msg:Image):
        cv_image = self._cvBridge.imgmsg_to_cv2(image_msg, "bgr8")
        if self._screen_rotate==0:
            self._fb[:]=cv_image
        else:
            self._fb[:]=cv2.rotate(cv_image, self._screen_rotate-1)
        
    def _requester(self, request:PlayMediaRequest):
        response = PlayMediaResponse()

        if (request.path_to_file == '') or os.path.exists(request.path_to_file):
            self._semaphore.acquire()
            self._is_played.value = 0
            self._is_cycled.value = request.is_cycled
            self._showing_file.value = request.path_to_file.encode()
            self._semaphore.release()

            while (request.is_blocking!=0) and (self._is_played.value==0):
                rospy.sleep(1/self._framerate)
            
            response.value = 0
            return response
        else:
            response.value = -1
            return response
    
    def drawError(self, error:str='') -> None:
        rospy.logerr(error)

        self._fb[:] = [255, 0, 0]
        h, w, c = self._height, self._width, self._color_channels
        img = np.zeros((w, h, c), np.uint8)

        font                   = cv2.FONT_HERSHEY_SIMPLEX
        bottomLeftCornerOfText = (10,w//2)
        fontScale              = 1
        fontColor              = (0,255,0)
        thickness              = 1
        lineType               = 2

        symbs_per_line = h//20
        for i in range(len(error)//symbs_per_line+1):
            s = error[i*symbs_per_line:i*symbs_per_line+symbs_per_line]
            cv2.putText(img,s, 
            tuple(bottomLeftCornerOfText), 
            font, 
            fontScale,
            fontColor,
            thickness,
            lineType)
            bottomLeftCornerOfText = (bottomLeftCornerOfText[0], bottomLeftCornerOfText[1]+40)

        if self._screen_rotate==0:
            self._fb[:]+=img
        else:
            self._fb[:]+=cv2.rotate(img, self._screen_rotate-1)

    def _player(self) -> None:
        prev_cap = ''
        while not rospy.is_shutdown():
            self._semaphore.acquire()
            current_file = self._showing_file.value.decode()
            is_played = self._is_played.value
            is_cycled = self._is_cycled.value
            self._semaphore.release()
            
            if current_file == '':
                if is_played == 0:
                    self._fb[:] = 0
                    self._semaphore.acquire()
                    self._is_played.value = 1
                    self._semaphore.release()
                continue
            ext = current_file.split('.')[-1]

            if ext in ['jpg', 'png']:  
                if is_played == 0:
                    img = cv2.imread(current_file)
                    try:
                        if self._screen_rotate==0:
                            self._fb[:self._height, :self._width, :]=img
                        else:
                            self._fb[:self._height, :self._width, :]=cv2.rotate(img, self._screen_rotate-1)
                    except Exception as e:
                        self.drawError(f"Couldn`t show image.\nDetails: {e}")

                    self._semaphore.acquire()
                    self._is_played.value = 1
                    self._semaphore.release()

            elif ext in ['mp4', 'mov', 'avi']:
                if (is_cycled == 0) and (is_played == 1):
                    continue

                if  prev_cap!=current_file:
                    cap = cv2.VideoCapture(current_file)
                    fps = rospy.Rate(cap.get(cv2.CAP_PROP_FPS))
                    prev_cap=current_file

                ret, src = cap.read()
                if not ret:
                    self._semaphore.acquire()
                    self._is_played.value = 1
                    self._semaphore.release()
                    cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                else:
                    if self._screen_rotate==0:
                        self._fb[:self._height, :self._width, :]=src
                    else:
                        self._fb[:self._height, :self._width, :]=cv2.rotate(src, self._screen_rotate-1)
                    fps.sleep()
 
            else:
                if is_played == 0:
                    self.drawError("Unknown extension of file")
                    self._semaphore.acquire()
                    self._is_played.value = 1

    def _touchscreen(self) -> None:
        flag = 0
        x=0
        y=0
        msg = Pose2D()
        self._touch.grab()
        while not rospy.is_shutdown():
            _,_,_ = select.select([self._touch], [], [])
            for event in self._touch.read():
                if event.type == evdev.ecodes.EV_ABS:
                    if event.code == 0:
                        x = event.value
                    elif event.code == 1:
                        y = event.value
                        flag = 1
                if flag==1:
                    flag = 0

                    rotate = radians(90*self._touchscreen_rotate)                    
                    x_ = x*cos(rotate)+y*sin(rotate)
                    y_ = -x*sin(rotate)+y*cos(rotate)

                    if self._touchscreen_rotate==1:
                        y_+=self._touchscreen_resolution_w
                    elif self._touchscreen_rotate==2:
                        y_+=self._touchscreen_resolution_h
                        x_+=self._touchscreen_resolution_w
                    elif self._touchscreen_rotate==3:
                        x_+=self._touchscreen_resolution_h

                    msg.x = int(x_/self._touchscreen_resolution_w*self._width)
                    msg.y = int(y_/self._touchscreen_resolution_h*self._height)
                    self._touchsreen_publisher.publish(msg)

    def stop(self, arg1=None, arg2=None) -> None:
        self._proccess_player.kill()
        self._proccess_player.join()

        if self._use_touch:
            self._touch.ungrab()

        self._fb[:]=0

        rospy.loginfo("display_driver SHUTDOWN")
        exit(0)


if __name__ == "__main__":
    rospy.init_node("display_driver")
    obj = DisplayDriver()
    rospy.spin()