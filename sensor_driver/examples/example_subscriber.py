# ========================================================= #
#                                                           #
#   Пример работы с пакетом sensor_driver на ЯП python      #
#                                                           #
# ========================================================= #

import rospy # библиотека для работы с ROS
from sensor_msgs.msg import BatteryState # тип сообщения BatteryState для топика с данными батареи
 
def callback(msg:BatteryState): # Функция callback для получения сообщений из топика
    print(f"Voltage: {msg.voltage}")
    print(f"Current: {msg.current}")
    print("------------------------")
    

rospy.init_node('example_subscriber') # создаем ROS-ноду
rospy.Subscriber("sensor_driver/bat", BatteryState, callback) # Подписываемся на топик с данными батареи
rospy.Timer(rospy.Duration(5), callback=lambda e: rospy.signal_shutdown("end")) # Таймер для завершения ROS через 5 секунд.
rospy.spin() # Зацикливаемся