# example_ears_controller.py

import rospy # Библиотека для работы с ROS
import time

# Подключаем тип сообщения для сервиса EarsSetAngle
from ears_controller.srv import EarsSetAngle, EarsSetAngleRequest

rospy.init_node("example_ears_controller_node") # Инициализируем ROS-ноду

# Создаем объект сервиса для работы с ушами
service_ears = rospy.ServiceProxy('EarsSetAngle', EarsSetAngle)

request = EarsSetAngleRequest() # Объект запроса

# Уши вперед
request.EarLeft = 90
request.EarRight = 90
service_ears(request)

time.sleep(1)

# Уши назад
request.EarLeft = -90
request.EarRight = -90
service_ears(request)

time.sleep(1)

# Начальное положение
request.EarLeft = 0
request.EarRight = 0
service_ears(request)

time.sleep(1)

# Попеременно дергаем ушами
k = 1
while not rospy.is_shutdown():
    request.EarLeft = 90*k
    request.EarRight = 90*(-k)
    service_ears(request)
    time.sleep(1)
    k=k*(-1)