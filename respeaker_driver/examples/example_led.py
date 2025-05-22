# ========================================================= #
#                                                           #
#   Пример работы с пакетом speakers_driver на ЯП python    #
#   Управление светодиодным кольцом на микрофонном модуле   #
#                                                           #
# ========================================================= #

from respeaker_driver.msg import SetColorManualLED # Тип сообщения для ручного задания цвета КАЖДОГО светодиода в отедьлности
from respeaker_driver.srv import SetBrightnessLED, SetBrightnessLEDRequest, SetBrightnessLEDResponse # Тип сообщения для сервиса установки яркости светодиодов
from respeaker_driver.srv import SetColorAllLED, SetColorAllLEDRequest, SetColorAllLEDResponse # Тип сообщения для сервиса установки одного цвета на все светодиоды разом
from respeaker_driver.srv import SetColorPaletteLED, SetColorPaletteLEDRequest, SetColorPaletteLEDResponse # Тип сообщения для сервиса установки палитры цветов для предустановленных режимов работы LED кольца
from respeaker_driver.srv import SetModeLED, SetModeLEDRequest, SetModeLEDResponse # Тип сообщения для сервиса установки режима работы LED-кольца

import rospy # библиотека для работы с ROS

rospy.init_node("example_led") # Инициализируем ROS-ноду

srv_SetBrightnessLED = rospy.ServiceProxy("/respeaker_driver/SetBrightnessLED", SetBrightnessLED) # Клиент для вызова сервиса SetBrightnessLED
srv_SetColorAllLED = rospy.ServiceProxy("/respeaker_driver/SetColorAllLED", SetColorAllLED) # Клиент для вызова сервиса SetColorAllLED
srv_SetColorPaletteLED = rospy.ServiceProxy("/respeaker_driver/SetColorPaletteLED", SetColorPaletteLED) # Клиент для вызова сервиса SetColorPaletteLED
srv_SetModeLED = rospy.ServiceProxy("/respeaker_driver/SetModeLED", SetModeLED) # Клиент для вызова сервиса SetModeLED
pub_SetColorManualLED = rospy.Publisher("/respeaker_driver/SetColorManualLED", SetColorManualLED, queue_size=1) # Паблишер для топика SetColorManualLED


# Устанавливаем яркость светодиодной подсветки микрофонного модуля
request = SetBrightnessLEDRequest()
request.brightness = 15 # Диапазон значений 0...31
response = SetBrightnessLEDResponse()
response = srv_SetBrightnessLED(request) # Вызываем сервис и сохраняем возвращенное значение
print("SetBrightnessLED", response.value)

# Устанавливаем палитру:

# Цвет A - синий [red, green, blue]
# Цвет Б - красный [reg, green, blue]
# Диапазон каждого канала (rgb): 0...255
# Изменяет цветовую палитру для режима trace и listen
# A - цвет, указывающий направление, откуда идет звук 
# B - цвет, заполняющий остальные светодиоды
# Для режима spin цвет a задает общую заливку, b не используется
# Для режиме speak и wait задает два цвета между которыми идет смена
request = SetColorPaletteLEDRequest()
request.colorA = [0,0,255] # Задаем цвет А - синий
request.colorB = [255,0,0] # Задаем цвет Б - красный
response = SetColorPaletteLEDResponse()
response = srv_SetColorPaletteLED(request) # Вызываем сервис и сохраняем возвращенное значение
print("SetColorPaletteLED", response.value)

# Режим 0 - все светодиоды выключены
request = SetModeLEDRequest()
request = 0 # Режим 0
response = SetModeLEDResponse()
response = srv_SetModeLED(request) # Вызываем сервис и сохраняем возвращенное значение
print("SetModeLED(0)", response.value)

rospy.sleep(1) # Ждём секунду

# Режим 1 - trace - режим "слежения" за голосом
# когда есть звук (голос) светодиоды зажигаются следующим образом:
# цветом B из color_palette заливаются все свотодиоды
# цветом A из color_palette зажигается один светодиод, указывающий направление, откуда идет звук
# если звука нет, через пару секунд светодиоды тухнут
request = SetModeLEDRequest()
request = 1 # Режим 1
response = SetModeLEDResponse()
response = srv_SetModeLED(request) # Вызываем сервис и сохраняем возвращенное значение
print("SetModeLED(1)", response.value)

rospy.sleep(10) # Ждём 10 секунд, чтоб рассмотреть режим работы

# Режим 2 - listen
# тоже самое, что trace, только светодиоды горят всегда, независимо от наличия голоса
request = SetModeLEDRequest()
request = 2 # Режим 2
response = SetModeLEDResponse()
response = srv_SetModeLED(request) # Вызываем сервис и сохраняем возвращенное значение
print("SetModeLED(2)", response.value)

rospy.sleep(10) # Ждём 10 секунд, чтоб рассмотреть режим работы

# Режим 3 - wait - эффект "ожидания"
# берет из color_palette цвета a и b
# окрашивает все светодиоды сначала одним цветом, потом плавно меняет на другой и так далее
request = SetModeLEDRequest()
request = 3 # Режим 3
response = SetModeLEDResponse()
response = srv_SetModeLED(request) # Вызываем сервис и сохраняем возвращенное значение
print("SetModeLED(3)", response.value)

rospy.sleep(10) # Ждём 10 секунд, чтоб рассмотреть режим работы

# Режим 4 - speak - эффект "говорения"
# берет из color_palette цвета A и B и закрашивает ими светодиоды чередованием
# переключает попеременно цвета
request = SetModeLEDRequest()
request = 4 # Режим 4
response = SetModeLEDResponse()
response = srv_SetModeLED(request) # Вызываем сервис и сохраняем возвращенное значение
print("SetModeLED(4)", response.value)

rospy.sleep(10) # Ждём 10 секунд, чтоб рассмотреть режим работы

# Режим 5 - spin
# берет из color_palette цвет A, заливает все светодиоды этим цветом
# начинает плавно переливаться по кругу, понижая яркость свтодиодов
request = SetModeLEDRequest()
request = 5 # Режим 5
response = SetModeLEDResponse()
response = srv_SetModeLED(request) # Вызываем сервис и сохраняем возвращенное значение
print("SetModeLED(5)", response.value)

rospy.sleep(10) # Ждём 10 секунд, чтоб рассмотреть режим работы

# Режим 0 - все светодиоды выключены
request = SetModeLEDRequest()
request = 0 # Режим 0
response = SetModeLEDResponse()
response = srv_SetModeLED(request) # Вызываем сервис и сохраняем возвращенное значение
print("SetModeLED(0)", response.value)

rospy.sleep(1) # Ждём секунду

# Заливка всех светодиодов одним цветом
request = SetColorAllLEDRequest()
request.r = 255 # Красный канал
request.g = 127 # Зеленый канал
request.b = 64 # Синий канал
response = SetColorAllLEDResponse()
response = srv_SetColorAllLED(request) # Вызываем сервис и сохраняем возвращенное значение
print("SetColorAllLED", response.value)

rospy.sleep(5) # Ждём

# Заливка всех светодиодов одним цветом
request = SetColorAllLEDRequest()
request.r = 0 # Красный канал
request.g = 0 # Зеленый канал
request.b = 64 # Синий канал
response = SetColorAllLEDResponse()
response = srv_SetColorAllLED(request) # Вызываем сервис и сохраняем возвращенное значение
print("SetColorAllLED", response.value)

rospy.sleep(5) # Ждём

# Установка цвета каждого светодиода в отдельности
# r,g,b - цвета в формате 1-байта 0...255
# нумерация светодиодов: расположить respeaker так, чтобы usb выход был снизу
# тогда если представить, что светодиоды - это циферблат, то
# 1й светодиод будет там, где 1-й час, 2й светодиод на 2 часа и т.д.
# последний, 12й светодиод, будет там, куда указывала бы часовая стрелка на 12 часов
# порядок нумерации, соотвественно, по часовой стрелке
msg = SetColorManualLED()
msg.colors = [255,0,0, # 1
              0,255,0, # 2
              0,0,255, # 3
              0,0,0, # 4
              255,255,255, # 5
              0,0,0, # 6
              128,128,0, # 7
              0,128,128, # 8
              128,0,128, # 9
              255,255,255, # 10
              0,0,255, # 11
              255,0,0 # 12
              ]
pub_SetColorManualLED.publish(msg)

rospy.sleep(5) # Ждём