import rospy
from display_driver_py.srv import PlayMedia, PlayMediaRequest, PlayMediaResponse

import os
import cv2
import numpy as np
import multiprocessing

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class DisplayController():
    def __init__(self) -> None:
        
        rospy.init_node("display_driver_py")

        self._height = rospy.get_param("~height", 1080)
        self._width = rospy.get_param("~width", 1080)
        self._color_channels = rospy.get_param("~color_channels", 3)
        self._framerate = rospy.get_param("~framerate", 60)
        self._fb_name = rospy.get_param("~fb_name", "/dev/fb0")

        self._blank_name = rospy.get_param("~blank_name", "__BLANK__")
        
        service_PlayMedia_name = rospy.get_param("~service_PlayMedia_name", "~PlayMedia")
        topic_PlayMedia_name = rospy.get_param("~topic_PlayMedia_name", "~PlayMedia")


        manager = multiprocessing.Manager()
        self._showing_file = manager.Value('showing_file', self._blank_name)
        self._is_played = manager.Value('is_played', 0)
        self._is_cycled = manager.Value('is_cycled', 0)
        
        self._fb = np.memmap(self._fb_name, dtype='uint8', mode='w+', shape=(self._height, self._width, self._color_channels))
        
        self._cvBridge = CvBridge()

        play = multiprocessing.Process(target=self._player, args=(), daemon=True)
        play.start()

        PlayMedia_subscriber = rospy.Subscriber(topic_PlayMedia_name, Image, callback=self._img_sub)
        PlayMedia_service = rospy.Service(service_PlayMedia_name, PlayMedia, self._requester)

        rospy.loginfo("display_driver_py INITED")
        rospy.spin()

    def _img_sub(self, image_msg:Image):
        cv_image = self._cvBridge.imgmsg_to_cv2(image_msg, "bgr8")
        self._fb[:]=cv2.rotate(cv_image, cv2.ROTATE_90_COUNTERCLOCKWISE)
        
    def _requester(self, request:PlayMediaRequest):
        response = PlayMediaResponse()
        if os.path.exists(request.path_to_media):
            self._is_played.value = 0
            self._is_cycled.value = request.is_cycled
            self._showing_file.value = request.path_to_media

            while (request.is_blocking!=0) and (self._is_played.value==0):
                rospy.sleep(1/self._framerate)
            
            response.value = 0
            return response
        else:
            response.value = -1
            return response
    
    def drawError(self, error:str='') -> None:
        print(f"ERROR: {error}")
        self.response.value = 1
        try:
            self._fb[:] = [255, 0, 0]
            h, w, c = self.fb_hwc.value
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
                # print(s, bottomLeftCornerOfText)
                cv2.putText(img,s, 
                tuple(bottomLeftCornerOfText), 
                font, 
                fontScale,
                fontColor,
                thickness,
                lineType)
                bottomLeftCornerOfText = (bottomLeftCornerOfText[0], bottomLeftCornerOfText[1]+40)

            self.fb[:]=cv2.rotate(img, cv2.ROTATE_90_COUNTERCLOCKWISE)
            # self.fb[:] = img
        except Exception as e:
            print(e)
        

    def _player(self) -> None:
        prev_cap = ''
        while True:
            current_file = self._showing_file.value
            
            if current_file == self._blank_name:
                if self._is_played.value == 0:
                    self._fb[:] = 0
                    self._is_played.value = 1

            ext = current_file.split('.')[-1]

            if ext in ['jpg', 'png']:  
                if self._is_played.value == 0:
                    img = cv2.imread(current_file)
                    try:
                        self._fb[:self._height, :self._width, :] = cv2.rotate(img, cv2.ROTATE_90_COUNTERCLOCKWISE)
                    except Exception as e:
                        self.drawError(f"Couldn`t show image.\nDetails: {e}")

                    self._is_played.value = 1


            elif ext in ['mp4', 'mov', 'avi']:
                if (self._is_cycled.value == 0) and (self._is_played.value == 1):
                    continue
                
                if  prev_cap!=current_file:
                    cap = cv2.VideoCapture(current_file)
                    fps = rospy.Rate(cap.get(cv2.CAP_PROP_FPS))
                    prev_cap=current_file

                ret, src = cap.read()
                if not ret:
                    self._is_played.value = 1
                    cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                else:
                    self._fb[:] = cv2.rotate(src, cv2.ROTATE_90_COUNTERCLOCKWISE)
                    # cv2.waitKey(20)
                    fps.sleep()

                    
            else:
                if self._is_played.value == 0:
                    self.drawError("Unknown extension of file")
                    self._is_played.value = 1
                        


if __name__ == "__main__":
    obj = DisplayController()