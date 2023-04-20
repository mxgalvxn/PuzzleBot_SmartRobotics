#!/usr/bin/env python

import matplotlib.pyplot as plt
import numpy as np
import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy

from geometry_msgs.msg import Twist

import time
import math
# Define robot parameters
wheel_radius = 0.05  # radius of wheels (m)
wheelbase = 0.2  # distance between wheels (m) (l)
dt = 0.001  # time step (s)
t = 0  # Total time (s)
v_max = 1  # maximum linear velocity (m/s)
w_max = np.pi / 2  # maximum angular velocity (rad/s)

# Initialize robot state
x = 0  # x-position (m)
y = 0  # y-position (m)
theta = 0  # orientation (rad)

# Define reference positions
positions = np.array([[0, 0], [0, 20], [20, 20], [20, 0]])
num_positions = positions.shape[0]

# Set desired velocities
v_desired = 0.5  # linear velocity (m/s)
w_desired = np.pi / 4  # angular velocity (rad/s)


twist = Twist()

vmax = 5

kpr = 3
kpt = 4



if __name__=="__main__":
    try:
# Simulate robot motion
        rospy.init_node('square_mover')
        pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        i = 1
        while i < num_positions:
            xd = positions[i,0]
            yd = positions[i,1]

            thetad = math.atan2((yd-y), (xd-x))
            d = math.sqrt((xd-x)**2 + (yd-y)**2)

            thetae = (theta - thetad)
            if thetae > math.pi:
                thetae = thetae - 2*math.pi
            elif thetae < -math.pi:
                thetae = thetae + 2*math.pi

            w = -kpr*thetae
            v = vmax*math.tanh(d*kpt/vmax)
                        # Compute control inputs
            vr = v_max + (wheelbase * w_max) / 2  # Esto ir a al PWM
            vl = v_max - (wheelbase * w_max) / 2  # Esto ira al PWM

            # Compute robot motion
            v = (vr + vl) / 2
            w = (vr - vl) / wheelbase
            # Compute robot motion
            vx = v * np.cos(theta)
            vy = v * np.sin(theta)
            # Integrate wrt time
            x = x + vx * dt
            y = y + vy * dt
            theta = theta + w * dt

            twist.linear.x = v
            twist.angular.z = theta
            pub.publish(twist)
            # Check if we've reached the current reference po   sition
            error = np.sqrt((x - positions[i, 0])**2 + (y - positions[i, 1])**2)
            if error < 0.05:
                i += 1

            # Plot robot motion
            # plt.plot(x, y)
            # plt.axis([-1, 1, -1, 1])
            # plt.draw()
            # plt.pause(0.001)

            t = t + dt

        plt.show()

    except rospy.ROSInterruptException:
        pass
