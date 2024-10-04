# example_display_controller_file.py

import rospy # Библиотека для работы с ROS
import time
import os

# Подключаем тип сообщения для сервиса displayControllerPlay
from display_controller.srv import displayControllerPlay

rospy.init_node("example_display_controller_file_node") # Инициализируем ROS-ноду

# Создаем объект сервиса работы с дисплеем
service_display_player = rospy.ServiceProxy('displayControllerPlay', displayControllerPlay)

script_path = os.path.dirname(os.path.abspath(__file__))

# Выводим изображение
service_display_player(script_path+'/pic.png')

time.sleep(1) # Ждём 1 секунду

# Запускаем воспроизведение видео
service_display_player(script_path+'/vid.mp4')