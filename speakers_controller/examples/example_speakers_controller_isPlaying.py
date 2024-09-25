# example_speakers_controller_isPlaying.py

import rospy # Библиотека для работы с ROS

# Подключаем тип сообщения для сервиса isPlaying
from speakers_controller.srv import isPlaying
rospy.init_node("example_speakers_controller_isPlaying_node") # Инициализируем ROS-ноду

# Создаем объект сервиса для проверки воспроизведения аудио в данный момент
service_isPlaying = rospy.ServiceProxy('isPlaying', isPlaying)

response = service_isPlaying() # response типа isPlayingResponse
if response.Data==0:
    print("Audio is NOT playing")
elif response.Data==1:
    print("Audio is playing")