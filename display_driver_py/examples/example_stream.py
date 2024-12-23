# ========================================================= #
#                                                           #
#   Пример работы с пакетом display_driver_py               #
#   Воспроизведение потокового медиа на ЯП python            #
#   (для работы примера дополнительно запустите пакет:      #
#   cv_camera, команда: rosrun cv_camera cv_camera_node     #       
#                                                           #
# ========================================================= #

import rospy # Библиотека для работы с ROS
import cv2   # Библиотека OpenCV

from cv_bridge import CvBridge # Конвертация изображения ROS<->OpenCV

# Подключаем тип сообщения для сервиса PlayMedia
from display_driver_py.srv import PlayMedia, PlayMediaRequest

# Подключаем тип сообщения Image
from sensor_msgs.msg import Image

cvBridge = CvBridge()

def img_proc(image_msg:Image):
    cv_image = cvBridge.imgmsg_to_cv2(image_msg, "bgr8")
    cv_image = cv2.resize(cv_image, (1080, 1080))
    image_pub.publish(cvBridge.cv2_to_imgmsg(cv_image, encoding="bgr8")) 


rospy.init_node('example_stream_node') # Инициализируем ROS-ноду
rospy.wait_for_service("/display_driver_py/PlayMedia") # Дожидаемся инициализации сервиса

image_sub = rospy.Subscriber("/cv_camera/image_raw", Image, callback=img_proc)      # Подписчик на топик потока с камеры
image_pub = rospy.Publisher("/display_driver_py/PlayMedia", Image, queue_size=1)    # Паблишер для отправки изображений в топик PlayMedia

# Создаем объект сервиса работы с дисплеем
service_PlayMedia = rospy.ServiceProxy('/display_driver_py/PlayMedia', PlayMedia)

request = PlayMediaRequest()
request.path_to_media = "__BLANK__"
request.is_blocking = 1
request.is_cycled = 0
service_PlayMedia(request) # Останавливаем воспроизведение медиа на экране

rospy.spin() # Зацикливаем работу