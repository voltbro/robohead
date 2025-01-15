# ========================================================= #
#                                                           #
#   Пример работы с пакетом display_driver                  #
#   Использование тачскрина на ЯП python                    #
#                                                           #
# ========================================================= #

import rospy # Библиотека для работы с ROS
import cv2   # Библиотека OpenCV
import numpy as np # Библиотека numpy

from cv_bridge import CvBridge # Конвертация изображения ROS<->OpenCV

# Подключаем тип сообщения для сервиса PlayMedia
from display_driver.srv import PlayMedia, PlayMediaRequest

# Подключаем тип сообщения Image
from sensor_msgs.msg import Image

# Подключаем тип сообщения Pose2D
from geometry_msgs.msg import Pose2D

cvBridge = CvBridge()


def draw_circle(msg:Pose2D):
        cv_image = np.zeros((1080,1080,3), np.uint8)
        x = int(msg.x)
        y = int(msg.y)
        cv2.circle(cv_image, (x, y), 100, (255,255,255), -1)

        image_pub.publish(cvBridge.cv2_to_imgmsg(cv_image, encoding="bgr8")) 

rospy.init_node('example_touch_node') # Инициализируем ROS-ноду
rospy.wait_for_service("/display_driver/PlayMedia") # Дожидаемся инициализации сервиса

image_pub = rospy.Publisher("/display_driver/PlayMedia", Image, queue_size=1) # Паблишер для отправки изображений в топик PlayMedia
touch_sub = rospy.Subscriber("/display_driver/touchscreen", Pose2D, callback=draw_circle)      # Подписчик на топик с координатами тачскрина

# Создаем объект сервиса работы с дисплеем
service_PlayMedia = rospy.ServiceProxy('/display_driver/PlayMedia', PlayMedia)

request = PlayMediaRequest()
request.path_to_file = ""
request.is_blocking = 1
request.is_cycled = 0
service_PlayMedia(request) # Останавливаем воспроизведение медиа на экране

rospy.spin() # Зацикливаем работу


