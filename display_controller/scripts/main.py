import rospy
from display_controller.srv import displayControllerPlay, displayControllerPlayResponse
import cv2
import numpy as np
import multiprocessing
import os
import time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class DisplayController():
    def __init__(self, heigh:int=1080, width:int=1080, channels:int=3,
                 node_name:str="display_controller_node", service_name:str="displayControllerPlay") -> None:
        manager = multiprocessing.Manager()
        self.showing_file = manager.Value('showing_file', "")
        self.fb_hwc = manager.Value('fb_hwc', (heigh, width, channels))
        # self.response = manager.Value('response', 0)
        # self.showing_file = multiprocessing.Value('i', "")
        # self.fb_hwc = multiprocessing.Value('i', (heigh, width, channels))
        self.response = multiprocessing.Value('i', 0)
        self.fb = np.memmap('/dev/fb0', dtype='uint8',mode='w+', shape=(heigh, width, channels))
        
        play = multiprocessing.Process(target=self.player, args=(), daemon=True)
        play.start()

        rospy.init_node(node_name)
        play_srv = rospy.Service(service_name, displayControllerPlay, self.requester)
        self._cvBridge = CvBridge()
        image_sub = rospy.Subscriber("head/display/raw_image", Image, callback=self.img_sub)
        print("Ready")
        rospy.spin()

    def img_sub(self, image_msg:Image):
        cv_image = self._cvBridge.imgmsg_to_cv2(image_msg, "bgr8")
        self.fb[:] = cv_image

    def requester(self, request:displayControllerPlay):
        self.showing_file.value = request.FileName
        time.sleep(0.2)
        return displayControllerPlayResponse(self.response.value)
    
    def drawError(self, error:str='') -> None:
        print(f"ERROR: {error}")
        self.response.value = 1
        try:
            self.fb[:] = [255, 0, 0]
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

            # self.fb[:]=cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)
            self.fb[:] = img
        except Exception as e:
            print(e)
        

    def player(self) -> None:
        last_value = ''
        while True:
            try:
                current_value = self.showing_file.value

                if (current_value=="__BLANK__"):
                    if (last_value!="__BLANK__"):
                        self.fb[:] = 0
                        last_value="__BLANK__"

                ext = current_value.split('.')[-1]

                if ext in ['jpg', 'png']:  
                    if last_value!=current_value:
                        last_value = current_value
                        path = current_value

                        if os.path.isfile(path):
                            src = cv2.imread(path)

                            try:
                                # self.fb[:1080, :1080] = cv2.rotate(src, cv2.ROTATE_90_CLOCKWISE)
                                self.fb[:] = src
                                self.response.value = 0
                            except Exception as e:
                                self.drawError(f"Couldn`t show image.\nDetails: {e}")
                        else:
                            self.drawError("File is not exists")


                elif ext in ['mp4', 'mov', 'avi']:
                    if last_value!=current_value:
                        last_value=current_value
                        path = current_value

                        if os.path.isfile(path):
                            try:
                                cap = cv2.VideoCapture(path)
                                fps = cap.get(cv2.CAP_PROP_FPS)
                                errFlag=False
                            except Exception as e:
                                self.drawError(f"Couldn`t open video.\nDetails: {e}")
                                errFlag=True

                        else:
                            self.drawError("File is not exists")
                            errFlag=True

                    if not errFlag:
                        ret, src = cap.read()
                        if not ret:
                            cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                        else:
                            try:
                                # self.fb[:] = cv2.rotate(src, cv2.ROTATE_90_CLOCKWISE)
                                self.fb[:] = src
                                self.response.value = 0
                                cv2.waitKey(20)
                            except Exception as e:
                                self.drawError(f"Couldn`t show frame video.\nDetails: {e}")
                                errFlag=True
                        
                else:
                    if last_value!=current_value:
                        last_value=current_value
                        self.drawError("Unknown extension of file")
                        # self.fb[:] = [0, 255, 0]
                        
            except BaseException as e:
                if last_value!=current_value:
                    last_value=current_value
                    self.drawError(f"Fatal Exception: {e}")
                    # self.fb[:] = [0, 0, 255]
                    

if __name__ == "__main__":
    DisplayController()