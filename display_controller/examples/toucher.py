import evdev, select, time

import rospy    # Библиотека для работы с ROS
import cv2      # Библиотека OpenCV
import numpy as np

from cv_bridge import CvBridge # Конвертация изображения ROS<->OpenCV

# Подключаем тип сообщения для сервиса displayControllerPlay
from display_controller.srv import displayControllerPlay

# Подключаем тип сообщения Image
from sensor_msgs.msg import Image

cvBridge = CvBridge()

rospy.init_node('example_display_controller_toucher_node') # Инициализируем ROS-ноду

image_pub = rospy.Publisher("head/display/raw_image", Image, queue_size=1) # Паблишер для отправки изображений в топик потока displayController
service_display_player = rospy.ServiceProxy('displayControllerPlay', displayControllerPlay)
service_display_player('__BLANK__') # Останавливаем воспроизведение любого видео


def getPixelsFromCoordinates(coo):
    x = coo[0]/4095*1079
    y = coo[1]/4095*1079

    return (1079-int(y), int(x))

last_coord = (0,0)
def print_circle(coords:tuple):
    global last_coord
    if ( ((last_coord[0]-coords[0])**2+(last_coord[1]-coords[1])**2)**0.5 > 50):
        last_coord = coords
        cv_image = np.zeros((1080,1080,3), np.uint8)
        cv2.circle(cv_image, coords, 200, (255,255,255), -1)

        image_pub.publish(cvBridge.cv2_to_imgmsg(cv_image, encoding="bgr8")) 
    return 0



for i in [0,1]:
    touch = evdev.InputDevice(f"/dev/input/event{i}")
    print(touch.name, f"/dev/input/event{i}")
    if ("waveshare" in str(touch.name).lower()):
        break

touch.grab()

coords = (0,0)
flag = 0
X=0
Y=0

while not rospy.is_shutdown():
    r,w,x = select.select([touch], [], [])
    for event in touch.read():
        if event.type == evdev.ecodes.EV_ABS:
            if event.code == 0:
                X = event.value
            elif event.code == 1:
                Y = event.value
                flag = 1
        if flag==1:
            flag = 0
            coords = getPixelsFromCoordinates((X, Y))
            print_circle(coords)

touch.ungrab()