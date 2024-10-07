import os
import sys
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

import INA219

import rospy
from sensor_msgs.msg import BatteryState



ina219 = INA219.INA219(addr=0x43)

def talker():
    pub = rospy.Publisher('head/bat', BatteryState, queue_size=10)
    rospy.init_node('sensor_driver', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    print("Sensor driver inited!")
    while not rospy.is_shutdown():
        msg = BatteryState()
        msg.voltage = ina219.getBusVoltage_V()
        msg.current = ina219.getCurrent_mA()/1000
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass