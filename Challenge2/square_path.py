#!/usr/bin/env python

import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy

from geometry_msgs.msg import Twist

import time

msg = "Corriendo square_path.py"

# Velocidad de movimiento
distance = 2

# Tiempo de espera entre cada giro


twist = Twist()

if __name__=="__main__":
    try:
        # Iniciar nodo
        rospy.init_node('square_mover')
        print(msg)

        # Crear publicador
        pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        time_zero = rospy.Time.now().to_sec()

        speed = distance/(rospy.Time.now().to_sec() - time_zero)
        twist.linear.x = speed
        twist.angular.z = 0.0
        pub.publish(twist)
        if(speed < .5):
            twist.linear.x = 0.0
            twist.angular.z = 2*3.1416/4
            time.sleep(1)
        

        # Mover en un cuadrad

    except rospy.ROSInterruptException:
        pass
