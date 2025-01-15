#!/usr/bin/env python

from sensor_driver_dependencies.INA219 import INA219

import rospy
from sensor_msgs.msg import BatteryState

class SensorDriver():
    def callback(self, e):
        msg = BatteryState()
        msg.voltage = self.ina219.getBusVoltage_V()
        msg.current = self.ina219.getCurrent_mA()/1000
        self.pub.publish(msg)

    def __init__(self):
        topic_name = rospy.get_param("~topic_name", "~bat")
        publish_rate = rospy.get_param("~publish_rate", 5)

        i2c_address = rospy.get_param("~i2c_address", 0x43)
        i2c_bus = rospy.get_param("~i2c_bus", 1)

        self.ina219 = INA219(i2c_bus=i2c_bus, addr=i2c_address)

        self.pub = rospy.Publisher(topic_name, BatteryState, queue_size=10)
        rospy.Timer(rospy.Duration(1/publish_rate), self.callback)

        rospy.loginfo("sensor_driver INITED")

if __name__ == '__main__':
    rospy.init_node('sensor_driver')
    SensorDriver()
    rospy.spin()
