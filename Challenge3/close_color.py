#!/usr/bin/env python

import matplotlib.pyplot as plt
import numpy as np
import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from std_msgs.msg import String

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
positions = np.array([[0, 0], [2, 0], [2, 2], [0, 2], [0,0]])
num_positions = positions.shape[0]

# Define gains for the PID controller
kpr = 1.4
kpt = .6


wr = 0.0
wl = 0.0

# Define callback functions for wheel velocities
def wl_callback(data):
    global wl
    wl = data.data

def wr_callback(data):
    global wr
    wr = data.data

def color_callback(data):
    global color
    color = data.data


# Set up ROS node and publishers/subscribers
rospy.init_node('square_mover')

nodeRate = 100
rate = rospy.Rate(nodeRate)

sub_color = rospy.Subscriber('colorSignal', string, color_callback)

twist = Twist()

color_value = 0

if (sub_color == "Amarillo"){
    color_value = .5
}
elif (sub_color == "Verde"){
    color_value = 1
}
elif(sub_color == "Rojo"){
        color_value = 0
}
vmax = .45 * color_value


if __name__=="__main__":
    try:

        pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

        sub_wl = rospy.Subscriber('wl', Float32, wl_callback)
        sub_wr = rospy.Subscriber('wr', Float32, wr_callback)
        # Simulate robot motion
        i = 1
        while i < num_positions:
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
            
            print('y actual = ', y, 'y deseada = ', yd)
            print('error =', error)
            

            # Compute wheel velocities from desired linear and angular velocities
            twist.linear.x = vref
            twist.angular.z = wref
            pub.publish(twist)

            # Check if the robot has reached the desired position
            if abs(error) < 0.01:
                i += 1
            t = t + dt
            rate.sleep()

    except rospy.ROSInterruptException:
        pass


