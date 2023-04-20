#!/usr/bin/env python

import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy

from geometry_msgs.msg import Twist

import time
import math

msg = "Corriendo square_path.py"
k = 10.0
distance = 3.0

# Velocidad de movimiento
max_speed = 1

# Tiempo de espera entre cada giro


twist = Twist()

div = 1000000000.0
if __name__=="__main__":
    try:
        # Iniciar nodo
        rospy.init_node('square_mover')
        print(msg)

        # Crear publicador
        pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

        time_zero = (rospy.get_rostime().nsecs)
        print((rospy.get_rostime().nsecs- time_zero)/div)
        time.sleep(1)
        tf = distance / max_speed
        t = (rospy.get_rostime().nsecs- time_zero)/div
        speed = max_speed
        pub.publish(twist)
        print(speed)
        i = 0
        while speed > 0 and i < 4:     
            t = (rospy.get_rostime().nsecs- time_zero)/div
            speed = max_speed*(1+math.tanh(k*(tf-t)))/2
            pub.publish(twist)
            print("vel: " + str(speed) + " time: " + str(t)+ " vuelta: " + str(i))
            twist.linear.x = speed
            twist.angular.z = 0.0
            if(speed < .1):
                twist.linear.x = 0.0
                twist.angular.z = 2*3.1416/4
                pub.publish(twist)
                time.sleep(1)
                i = i+1
            

        # Mover en un cuadrad

    except rospy.ROSInterruptException:
        pass
