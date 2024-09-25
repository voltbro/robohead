# example_speakers_controller_setVolume.py

import rospy # Библиотека для работы с ROS

# Подключаем тип сообщения для сервиса isPlaying
from speakers_controller.srv import setVolume, setVolumeRequest
rospy.init_node("example_speakers_controller_setVolume_node") # Инициализируем ROS-ноду

# Создаем объект сервиса для установки громкости воспроизведения
service_setVolume = rospy.ServiceProxy('setVolume', setVolume)

request = setVolumeRequest()
request.Volume = 30 # Громкость 30 единиц, диапазон 0..100
service_setVolume(request)