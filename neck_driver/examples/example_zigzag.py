# ========================================================= #
#                                                           #
#   Пример работы с пакетом neck_driver на ЯП python        #
#                                                           #
# ========================================================= #

import rospy    # Библиотека для работы с ROS

# Подключаем типы сообщений сервиса NeckSetAngle:
from neck_driver.srv import NeckSetAngle, NeckSetAngleRequest

# Инициализируем ROS-ноду:
rospy.init_node("ears_driver_example_zigzag")

# Дожидаемся инициализации сервиса
rospy.wait_for_service("/neck_driver/NeckSetAngle")

# Создаем объект сервиса для работы с шеей:
service_neck = rospy.ServiceProxy('/neck_driver/NeckSetAngle', NeckSetAngle)

# Создаем объект запроса:
request = NeckSetAngleRequest()

# Опускаем голову вниз за 1 сек, вызов сервиса блокирующий:
request.vertical_angle = -30
request.horizontal_angle = 0
request.duration = 1
request.is_blocking = 1
service_neck(request)

# Поднимаем голову вверх за 2 сек, вызов сервиса неблокирующий (поэтому необходим rospy.sleep):
request.vertical_angle = 30
request.horizontal_angle = 0
request.duration = 2
request.is_blocking = 0
service_neck(request)

rospy.sleep(2)

# Возвращаем голову в стандартное положение за 1 сек, вызов сервиса блокирующий:
request.vertical_angle = 0
request.horizontal_angle = 0
request.duration = 1
request.is_blocking = 1
service_neck(request)

# Двигем головой в стороны
request.vertical_angle = 0
request.horizontal_angle = -30
request.duration = 1
request.is_blocking = 1
service_neck(request)

request.vertical_angle = 0
request.horizontal_angle = 30
request.duration = 2
request.is_blocking = 1
service_neck(request)

# Устанавливаем начальное положение
request.vertical_angle = 0
request.horizontal_angle = 0
request.duration = 1
request.is_blocking = 1
service_neck(request)