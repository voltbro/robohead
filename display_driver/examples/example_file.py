# ========================================================= #
#                                                           #
#   Пример работы с пакетом display_driver                  #
#   Воспроизведение медиа из файла на ЯП python             #
#                                                           #
# ========================================================= #

import rospy # Библиотека для работы с ROS
import os    # Библиотека для работы с ОС

# Подключаем тип сообщения для сервиса PlayMedia
from display_driver.srv import PlayMedia, PlayMediaRequest

rospy.init_node("example_file_node") # Инициализируем ROS-ноду

rospy.wait_for_service("/display_driver/PlayMedia") # Дожидаемся инициализации сервиса

# Создаем объект сервиса работы с дисплеем
service_PlayMedia = rospy.ServiceProxy('/display_driver/PlayMedia', PlayMedia)

script_path = os.path.dirname(os.path.abspath(__file__)) # Получаем путь текущей директории
print(script_path) # Выводим путь до текущей директории

request = PlayMediaRequest()

request.path_to_file = script_path+"/vid.mp4" # Абсолютный путь до файла pic.png в текущей директории
request.is_blocking = 1 # Вызов блокирующий (вызов будет завершен, после воспроизведения всего видео)
request.is_cycled = 0 # Зацикливания воспроизведения нет (для изображения не имеет смысла)
result = service_PlayMedia(request) # Вызов сервиса
print(result) # Выводим результат вызова сервиса

request.path_to_file = script_path+"/pic.png" # Абсолютный путь до файла pic.png в текущей директории
request.is_blocking = 0 # Вызов неблокирующий (вызов будет завершен, не дожидаясь вывода на экран)
request.is_cycled = 0 # Зацикливания воспроизведения нет (для изображения не имеет смысла)
result = service_PlayMedia(request) # Вызов сервиса
print(result) # Выводим результат вызова сервиса