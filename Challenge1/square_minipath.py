#!/usr/bin/env python

import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy

from geometry_msgs.msg import Twist

import time

msg = "Corriendo square_minipath.py"

# Velocidad de movimiento
speed = 4

# Tiempo de espera entre cada giro
turn_time = 1

# Tiempo de espera entre cada movimiento hacia adelante
forward_time = 1
twist = Twist()

def move_forward():
    # Crear mensaje Twist y asignar valores

    twist.linear.x = speed
    twist.angular.z = 0.0

    # Publicar mensaje
    pub.publish(twist)

    # Esperar
    time.sleep(forward_time)

def turn_left():
    
    # Crear mensaje Twist y asignar valores
    twist.linear.x = 0.0
    twist.angular.z = 2 * 3.14159265359 / 4 

    # Publicar mensaje
    pub.publish(twist)

    # Esperar
    time.sleep(turn_time)

def stop():
    # Crear mensaje Twist y asignar valores
    twist.linear.x = 0.0
    twist.angular.z = 0.0
    
    # Publicar mensaje
    pub.publish(twist)
    time.sleep(1)

def side():
    move_forward()
    move_forward()
    stop()
    turn_left()
    stop()

def move_square():
    for i in range (4):
        side()
        i+=1
    # Parar
    stop()

if __name__=="__main__":
    try:
        # Iniciar nodo
        rospy.init_node('square_mover')
        print(msg)

        # Crear publicador
        pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

        # Mover en un cuadrado
        move_square()

    except rospy.ROSInterruptException:
        pass
