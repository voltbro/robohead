# example_speakers_controller_stopPlaying.py

import rospy # Библиотека для работы с ROS

# Подключаем тип сообщения для сервиса stopPlaying
from speakers_controller.srv import stopPlaying
rospy.init_node("example_speakers_controller_stopPlaying_node") # Инициализируем ROS-ноду

# Создаем объект сервиса для проверки воспроизведения аудио в данный момент
service_stopPlaying = rospy.ServiceProxy('stopPlaying', stopPlaying)

response = service_stopPlaying() # response типа stopPlayingResponse