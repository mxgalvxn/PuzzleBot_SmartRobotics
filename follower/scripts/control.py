#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32, String
from geometry_msgs.msg import Twist
import math as mt

error = 0.0
signal = "GREEN"

def lane_error_callback(data):
    global error
    error = data.data

def traffic_sign_callback(data):
    global signal
    signal = data.data


if __name__=='__main__':
    try:
        rospy.init_node("follow_it", anonymous=True)
        rate = rospy.Rate(100)

        sub_vision_error = rospy.Subscriber("/lane_error", Float32, lane_error_callback)
        sub_signal = rospy.Subscriber("/signalTraffic", String, traffic_sign_callback)

        pub_velocity = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        pV = Twist()
        pV.linear.x = 0.0
        pV.linear.y = 0.0
        pV.linear.z = 0.0
        pV.angular.x = 0.0
        pV.angular.y = 0.0
        pV.angular.z = 0.0

        last_error = 0.0
        integral = 0.0

        kp = 0.6
        kd = 2 * mt.sqrt(kp)

        #t = 0
        #dt = 0.1

        last_time = rospy.get_time()

        while not rospy.is_shutdown():
            if (signal == "YELLOW" or signal == "CONSTRUCTION" or signal == "GIVEWAY"):
                signal_value = .5
            elif (signal == "GREEN" or signal == "FORWARD"):
                signal_value = 1.0
            elif(signal == "RED" or signal == "STOP"):
                signal_value = 0
            elif( signal == "RIGHT"):
                pV.angular.z = -1.57  # 90 degrees in radians (clockwise)
                pV.linear.x = 0.2
                pub_velocity.publish(pV)
                rospy.sleep(2)
            elif signal == "LEFT":
                pV.angular.z = 1.57  # 90 degrees in radians (counter-clockwise)
                pV.linear.x = 0.2
                pub_velocity.publish(pV)
                rospy.sleep(2)
            else: 
                signal_value = 1
            vmax = .45 * signal_value

            
            # Adjust the sleep duration as needed

            current_t = rospy.get_time()
            dt = current_t - last_time
            if (dt == 0):
                dt = .00001
            last_time = current_t

            prop = kp * error
            der = kd * (error - last_error) / dt

            w = prop + der

            print(error)

            if w >= 0.35:
                w = 2.4
            elif w <= -0.35:
                w = -2.2
            
            if error > 55 or error < -55.0:
                pV.linear.x = 0.10 * vmax
            else:
                pV.linear.x = 0.15 * vmax
                w = 0
            
            pV.angular.z = w * vmax
            pub_velocity.publish(pV)

            last_error = error
        
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
