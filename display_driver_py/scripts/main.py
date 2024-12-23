import rospy
from display_driver_py.srv import PlayMedia, PlayMediaRequest, PlayMediaResponse

import os
import cv2
import numpy as np
import multiprocessing

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from ctypes import c_char
import signal

class DisplayController():

    def __init__(self) -> None:
        rospy.init_node("display_driver_py")

        self._height = rospy.get_param("~height", 1080)
        self._width = rospy.get_param("~width", 1080)
        self._color_channels = rospy.get_param("~color_channels", 3)
        self._framerate = rospy.get_param("~framerate", 60)
        self._fb_name = rospy.get_param("~fb_name", "/dev/fb0")
        self._rotate = rospy.get_param("~rotate", 0)

        self._blank_name = rospy.get_param("~blank_name", "__BLANK__")
        
        service_PlayMedia_name = rospy.get_param("~service_PlayMedia_name", "~PlayMedia")
        topic_PlayMedia_name = rospy.get_param("~topic_PlayMedia_name", "~PlayMedia")

        self._semaphore = multiprocessing.Semaphore(1)
        self._showing_file = multiprocessing.Array(c_char, 4096)
        self._showing_file.value = self._blank_name.encode()
        self._is_played = multiprocessing.Value('i', 0)
        self._is_cycled = multiprocessing.Value('i', 0)
        
        self._fb = np.memmap(self._fb_name, dtype='uint8', mode='w+', shape=(self._height, self._width, self._color_channels))
        
        self._cvBridge = CvBridge()

        self._proccess_player = multiprocessing.Process(target=self._player, args=(), daemon=True)
        self._proccess_player.start()
        signal.signal(signal.SIGINT, self.stop)

        PlayMedia_subscriber = rospy.Subscriber(topic_PlayMedia_name, Image, callback=self._img_sub)
        PlayMedia_service = rospy.Service(service_PlayMedia_name, PlayMedia, self._requester)

        rospy.loginfo("display_driver_py INITED")
        rospy.spin()

    def _img_sub(self, image_msg:Image):
        cv_image = self._cvBridge.imgmsg_to_cv2(image_msg, "bgr8")
        self._fb[:]=cv2.rotate(cv_image, self._rotate)
        
    def _requester(self, request:PlayMediaRequest):
        response = PlayMediaResponse()

        if (request.path_to_media == self._blank_name) or os.path.exists(request.path_to_media):
            self._semaphore.acquire()
            self._is_played.value = 0
            self._is_cycled.value = request.is_cycled
            self._showing_file.value = request.path_to_media.encode()
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

        self._fb[:]+=cv2.rotate(img, self._rotate)

    def _player(self) -> None:
        prev_cap = ''
        while True:
            self._semaphore.acquire()
            current_file = self._showing_file.value.decode()
            is_played = self._is_played.value
            is_cycled = self._is_cycled.value
            self._semaphore.release()
            
            if current_file == self._blank_name:
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
                        self._fb[:self._height, :self._width, :] = cv2.rotate(img, self._rotate)
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
                    self._fb[:] = cv2.rotate(src, self._rotate)
                    fps.sleep()
 
            else:
                if is_played == 0:
                    self.drawError("Unknown extension of file")
                    self._semaphore.acquire()
                    self._is_played.value = 1


    def stop(self, arg1=None, arg2=None):
        self._proccess_player.kill()
        self._proccess_player.join()

        self._fb[:]=0
        exit(0)


if __name__ == "__main__":
    obj = DisplayController()