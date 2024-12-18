#!/usr/bin/env python
import rospy
from sensor_msgs.msg import BatteryState
 
def callback(msg:BatteryState):
    print(f"Voltage: {msg.voltage}")
    print(f"Current: {msg.current}")
    print("------------------------")
    
def listener():
    rospy.init_node('example_subscriber')

    rospy.Subscriber("sensor_driver/bat", BatteryState, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()