# example_display_controller_stream.py

import rospy # Библиотека для работы с ROS
import cv2 # Библиотека OpenCV

from cv_bridge import CvBridge # Конвертация изображения ROS<->OpenCV

# Подключаем тип сообщения для сервиса displayControllerPlay
from display_controller.srv import displayControllerPlay

# Подключаем тип сообщения Image
from sensor_msgs.msg import Image

cvBridge = CvBridge()

def img_proc(image_msg:Image):
    cv_image = cvBridge.imgmsg_to_cv2(image_msg, "bgr8")
    cv_image = cv2.resize(cv_image, (1080, 1080))
    image_pub.publish(cvBridge.cv2_to_imgmsg(cv_image, encoding="bgr8")) 


rospy.init_node('example_display_controller_stream_node') # Инициализируем ROS-ноду

image_sub = rospy.Subscriber("cv_camera/image_raw", Image, callback=img_proc) # Подписчик на топик потока с камеры
image_pub = rospy.Publisher("head/display/raw_image", Image, queue_size=1) # Паблишер для отправки изображений в топик потока displayController
service_display_player = rospy.ServiceProxy('displayControllerPlay', displayControllerPlay)
service_display_player('__BLANK__') # Останавливаем воспроизведение любого видео
rospy.spin() # Зацикливаем работу