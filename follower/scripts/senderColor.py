#!/usr/bin/env python
import rospy
from std_msgs.msg import String

if __name__ == '__main__':
    pub1 = rospy.Publisher("/signalTraffic", String, queue_size = 10)
    rospy.init_node("signal_generator")
    rate = rospy.Rate(5)
    signal = "GREEN"

    while not rospy.is_shutdown():
       
       pub1.publish(signal)  
       rate.sleep()    
