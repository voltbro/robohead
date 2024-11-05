# example_neck_controller_zigzag.py

import rospy # Библиотека для работы с ROS
import time

# Подключаем тип сообщения для сервиса NeckSetAngle
from neck_controller.srv import NeckSetAngle, NeckSetAngleRequest

rospy.init_node("example_neck_controller_zigzag") # Инициализируем ROS-ноду

# Создаем объект сервиса работы с шеей
service_neck = rospy.ServiceProxy('NeckSetAngle', NeckSetAngle)

request = NeckSetAngleRequest() # Объект запроса

# Поднимаем голову вверх
request.AngleV = 30
request.AngleH = 0
request.TimeF = 0.7
service_neck(request)

time.sleep(0.7) # Ждём 0.7 секунды, пока голова достигнет положения

# Опускаем голову вниз
request.AngleV = -30
request.AngleH = 0
request.TimeF = 1.2
service_neck(request)

time.sleep(1.2) # Ждём 1.2 секунды, пока голова достигнет положения

# Возвращаем голову в начальное положение
request.AngleV = 0
request.AngleH = 0
request.TimeF = 0.5
service_neck(request)

time.sleep(0.5) # Ждём 0.5 секунды, пока голова достигнет положения

# Поворачиваем голову влево
request.AngleV = 0
request.AngleH = -30
request.TimeF = 1
service_neck(request)

time.sleep(1)

# Поворачиваем голову вправо
request.AngleV = 0
request.AngleH = 30
request.TimeF = 1
service_neck(request)

time.sleep(1)

# Возвращаем голову в начальное положение
request.AngleV = 0
request.AngleH = 0
request.TimeF = 0.5
service_neck(request)