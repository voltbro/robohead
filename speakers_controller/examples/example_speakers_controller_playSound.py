# example_speakers_controller_playSound.py

import rospy # Библиотека для работы с ROS

# Подключаем тип сообщения для сервиса playSound
from speakers_controller.srv import playSound, playSoundRequest
rospy.init_node("example_speakers_controller_playSound_node") # Инициализируем ROS-ноду

# Создаем объект сервиса для воспроизведения аудио-файла
service_playSound = rospy.ServiceProxy('playSound', playSound)

request = playSoundRequest()
request.FileName = "/home/user/examples/speakers_controller/audio.mp3" # Путь к аудио-файлу
request.IsBreakable = 0 # Блокирующий ли вызов сервиса:
# 0 - Воспроизведение будет выполняться ассинхронно, и не окажет влияния на текущую программу
# 1 - Дальнейшая программа не будет выполняться далее, пока не завершится воспроизведение аудио
service_playSound(request)