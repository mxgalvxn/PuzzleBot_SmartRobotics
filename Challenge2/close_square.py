#!/usr/bin/env python

import matplotlib.pyplot as plt
import numpy as np
import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

# Define callback functions for wheel velocities
def wl_callback(data):
    global wl
    wl = data.data

def wr_callback(data):
    global wr
    wr = data.data

# Set up ROS node and publishers/subscribers

if __name__=="__main__":
    try:
        rospy.init_node('square_mover')

        nodeRate = 100
        rate = rospy.Rate(nodeRate)


        twist = Twist()
        vmax = .2

        # Define robot parameters
        wheel_radius = 0.05  # radius of wheels (m)
        wheelbase = 0.19  # distance between wheels (m) (l)
        dt = 0.01  # time step (s)
        t = 0  # Total time (s)
        v_max = 1  # maximum linear velocity (m/s)
        w_max = np.pi / 2  # maximum angular velocity (rad/s)
        positions = np.array([[0, 0], [2, 0], [2, 2], [0, 2], [0,0]])
        num_positions = positions.shape[0]

        pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

        sub_wl = rospy.Subscriber('/wl', Float32, wl_callback)
        sub_wr = rospy.Subscriber('/wr', Float32, wr_callback)
        
        # wl = rospy.wait_for_message('/wl', Twist, timeout=1)
        # wr = rospy.wait_for_message('/wr', Twist, timeout=1)
        x = 0.0  # x-position (m)
        y = 0.0  # y-position (m)
        theta = 0  # orientation (rad)

        kpr = 1.4
        kpt = 4.5


        wr = 0.0
        wl = 0.0
        # Simulate robot motion
        i = 1
        while i < num_positions:

            # print('wl = ', wl)
            # print()
            # print('wr = ', wr)
            # print()

            xd = positions[i,0]
            yd = positions[i,1]

            # Get current position and orientation
            thetad = math.atan2((yd-y), (xd-x))
            error = math.sqrt((xd-x)**2 + (yd-y)**2)

            thetae = (theta - thetad)
            if thetae > math.pi:
                thetae = thetae - 2*math.pi
            elif thetae < -math.pi:
                thetae = thetae + 2*math.pi

            wref = -kpr * thetae
            vref = vmax*math.tanh(error*kpt/vmax)

            vr = vref + (wheelbase*wref)/2
            vl = vref - (wheelbase*wref)/2
            
            vref = (vr + vl)/2


            v_real = wheel_radius* (wr+wl)/2

            w_real = wheel_radius* (wr-wl)/wheelbase

            vx = v_real * math.cos(theta)
            vy = v_real * math.sin(theta)

            x = x + vx*dt
            y = y + vy*dt
            theta = theta + w_real * dt
            
            # print('y actual = ', y, 'y deseada = ', yd)
            # print()
            # print('x actual = ', x, 'x deseada = ', xd)
            # print()
            # print('error =', error)
            # print()
            

            # Compute wheel velocities from desired linear and angular velocities
            twist.linear.x = vref
            twist.angular.z = wref
            pub.publish(twist)

            # Check if the robot has reached the desired position
            if abs(error) < 0.1:
                i += 1
            t = t + dt
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
