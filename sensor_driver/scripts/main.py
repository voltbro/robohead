#!/usr/bin/env python

from sensor_driver_dependencies.INA219 import INA219

import rospy
from sensor_msgs.msg import BatteryState

class SensorDriver():
    def __init__(self):
        rospy.init_node('sensor_driver')

        topic_name = rospy.get_param("~topic_name", "~bat")
        publish_rate = rospy.get_param("~publish_rate", 5)

        i2c_address = rospy.get_param("~i2c_address", 0x43)
        i2c_bus = rospy.get_param("~i2c_bus", 1)

        ina219 = INA219(i2c_bus=i2c_bus, addr=i2c_address)

        pub = rospy.Publisher(topic_name, BatteryState, queue_size=10)
        rate = rospy.Rate(publish_rate)

        rospy.loginfo("sensor_driver INITED")

        msg = BatteryState()
        while not rospy.is_shutdown():
            msg.voltage = ina219.getBusVoltage_V()
            msg.current = ina219.getCurrent_mA()/1000
            pub.publish(msg)
            rate.sleep()

if __name__ == '__main__':
    try:
        obj = SensorDriver()
    except rospy.ROSInterruptException:
        pass