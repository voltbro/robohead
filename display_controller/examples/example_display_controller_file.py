# example_display_controller_file.py

import rospy # Библиотека для работы с ROS
import time

# Подключаем тип сообщения для сервиса displayControllerPlay
from display_controller.srv import displayControllerPlay

rospy.init_node("example_display_controller_file_node") # Инициализируем ROS-ноду

# Создаем объект сервиса работы с дисплеем
service_display_player = rospy.ServiceProxy('displayControllerPlay', displayControllerPlay)


# Выводим изображение
service_display_player('/home/user/examples/display_controller/pic.png')

time.sleep(1) # Ждём 1 секунду

# Запускаем воспроизведение видео
service_display_player('/home/user/examples/display_controller/vid.mp4')