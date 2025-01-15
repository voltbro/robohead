import rospy # Библиотека для работы с ROS
import os    # Библиотека для работы с ОС

# Подключаем тип сообщения для сервиса PlayAudio
from speakers_driver.srv import PlayAudio, PlayAudioRequest, PlayAudioResponse
# Подключаем тип сообщения для сервиса SetVolume
from speakers_driver.srv import SetVolume, SetVolumeRequest, SetVolumeResponse
# Подключаем тип сообщения для сервиса GetVolume
from speakers_driver.srv import GetVolume, GetVolumeRequest, GetVolumeResponse

rospy.init_node("example_file_node") # Инициализируем ROS-ноду

rospy.wait_for_service("/speakers_driver/PlayAudio") # Дожидаемся инициализации сервиса
rospy.wait_for_service("/speakers_driver/SetVolume") # Дожидаемся инициализации сервиса
rospy.wait_for_service("/speakers_driver/GetVolume") # Дожидаемся инициализации сервиса

# Создаем объект сервиса
service_PlayAudio = rospy.ServiceProxy('/speakers_driver/PlayAudio', PlayAudio)
service_SetVolume = rospy.ServiceProxy('/speakers_driver/SetVolume', SetVolume)
service_GetVolume = rospy.ServiceProxy('/speakers_driver/GetVolume', GetVolume)

script_path = os.path.dirname(os.path.abspath(__file__)) + '/' # Получаем путь текущей директории
print(script_path) # Выводим путь до текущей директории

request = PlayAudioRequest()
request.path_to_file = script_path+"file.mp3" # Абсолютный путь до файла file.mp3 в текущей директории
request.is_blocking = 1 # Вызов блокирующий (вызов будет завершен, после воспроизведения всего аудио)
request.is_cycled = 0 # Зацикливания воспроизведения нет 
result = service_PlayAudio(request) # Вызов сервиса
print(result) # Выводим результат вызова сервиса

request = SetVolumeRequest()
request.volume = 50
result = service_SetVolume(request)
print(result)

request = GetVolumeRequest()
result = service_GetVolume(request)
print(result)

request = PlayAudioRequest()
request.path_to_file = script_path+"file.mp3" # Абсолютный путь до файла file.mp3 в текущей директории
request.is_blocking = 1 # Вызов блокирующий (вызов будет завершен, после воспроизведения всего аудио)
request.is_cycled = 0 # Зацикливания воспроизведения нет 
result = service_PlayAudio(request) # Вызов сервиса