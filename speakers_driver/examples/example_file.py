# ========================================================= #
#                                                           #
#   Пример работы с пакетом speakers_driver на ЯП python    #
#                                                           #
# ========================================================= #

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
print("Script path:", script_path) # Выводим путь до текущей директории

request = SetVolumeRequest() # сообщение для вызова сервиса SetVolume
request.volume = 30 # Устанавливаем громкость 30%
result:SetVolumeResponse = service_SetVolume(request) # Вызываем сервис SetVolume
print("call SetVolume result:", result.value) # печатаем результат вызова сервиса

request = GetVolumeRequest() # сообщение для вызова сервиса GetVolume
result:GetVolumeResponse = service_GetVolume(request) # Вызываем сервис GetVolume
print("call GetVolume result (current volume):", result.value) # Печатаем результат вызова сервиса (текущую громкость)

request = PlayAudioRequest() # сообщение для вызова сервиса PlayAudio
request.path_to_file = script_path+"file.mp3" # Абсолютный путь до файла file.mp3 в текущей директории
request.is_blocking = 1 # Вызов блокирующий (вызов будет завершен, после воспроизведения всего аудио)
request.is_cycled = 0 # Зацикливания воспроизведения нет 
result:PlayAudioResponse = service_PlayAudio(request) # Вызов сервиса
print("call PlayAudio result:", result.value) # Выводим результат вызова сервиса

request = SetVolumeRequest() # сообщение для вызова сервиса SetVolume
request.volume = 50 # Устанавливаем громкость 50%
result:SetVolumeResponse = service_SetVolume(request) # Вызываем сервис SetVolume
print("call SetVolume result:", result.value) # печатаем результат вызова сервиса

request = PlayAudioRequest()
request.path_to_file = script_path+"file.mp3" # Абсолютный путь до файла file.mp3 в текущей директории
request.is_blocking = 0 # Вызов НЕблокирующий (вызов функции вернет результат ДО завершения воспроизведения)
request.is_cycled = 1 # Зацикливание воспроизведения включено
result:PlayAudioResponse = service_PlayAudio(request) # Вызов сервиса
print("call PlayAudio result:", result.value) # Выводим результат вызова сервиса

rospy.sleep(3) # "Засыпаем" на 2 секунды, пока воспроизводится аудио

request = PlayAudioRequest()
request.path_to_file = "" # Пустое поле для принудительной остановки текущего воспроиведения
request.is_blocking = 1 # Вызов блокирующий
request.is_cycled = 0 # Зацикливание воспроизведения включено
result:PlayAudioResponse = service_PlayAudio(request) # Вызов сервиса
print("call PlayAudio result:", result.value) # Выводим результат вызова сервиса
