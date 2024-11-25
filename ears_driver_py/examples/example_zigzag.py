# ========================================================= #
#                                                           #
#   Пример работы с пакетом ears_driver_py на ЯП python     #
#                                                           #
# ========================================================= #

import rospy    # Библиотека для работы с ROS

# Подключаем типы сообщений сервиса EarsSetAngle:
from ears_driver_py.srv import EarsSetAngle, EarsSetAngleRequest

# Инициализируем ROS-ноду:
rospy.init_node("ears_driver_py_example_zigzag") 

# Создаем объект сервиса для работы с ушами:
service_ears = rospy.ServiceProxy('/ears_driver_py/EarsSetAngle', EarsSetAngle)

# Создаем объект запроса:
request = EarsSetAngleRequest()

# Устанавливаем положение обоих ушей вперед до упора:
request.left_ear_angle = 90
request.right_ear_angle = 90
service_ears(request)

rospy.sleep(1)

# Уши назад
request.left_ear_angle = -90
request.right_ear_angle = -90
service_ears(request)

rospy.sleep(1)

# Начальное положение
request.left_ear_angle = 0
request.right_ear_angle = 0
service_ears(request)

rospy.sleep(1)

# Попеременно дергаем ушами
k = 1
for _ in range(5):
    request.left_ear_angle = 90*k
    request.right_ear_angle = 90*(-k)
    service_ears(request)
    rospy.sleep(1)
    k=k*(-1)

# Устанавливаем начальное положение
request.left_ear_angle = 0
request.right_ear_angle = 0
service_ears(request)