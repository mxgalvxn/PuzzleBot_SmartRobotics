#!/usr/bin/env python

import matplotlib.pyplot as plt
import numpy as np
import roslib
import rospy
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, String

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
kpr = 26
kpt = 18

# Define variables for traffic light state
traffic_light_state = "Green"
traffic_light_colors = ["Red", "Yellow", "Green"]
red_light_threshold = 0.05
yellow_light_threshold = 0.1

wr = 0.0
wl = 0.0

# Define callback functions for wheel velocities and traffic light state
def wl_callback(data):
    global wl
    wl = data.data

def wr_callback(data):
    global wr
    wr = data.data

def traffic_light_callback(data):
    global traffic_light_state
    traffic_light_state = data.data

# Set up ROS node and publishers/subscribers
rospy.init_node('square_mover')

nodeRate = 100
rate = rospy.Rate(nodeRate)

twist = Twist()
vmax = .45

if __name__=="__main__":
    try:
        pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        sub_wl = rospy.Subscriber('wl', Float32, wl_callback)
        sub_wr = rospy.Subscriber('wr', Float32, wr_callback)
        sub_traffic_light = rospy.Subscriber('color_pub', String, traffic_light_callback)

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

                        # Check traffic light
            color = rospy.wait_for_message('color_pub', String)
            if color.data == "Red":
                twist.linear.x = 0
                twist.angular.z = 0
                pub.publish(twist)
                rospy.loginfo("STOPPED at red light")
                while color.data != "Green":
                    color = rospy.wait_for_message('color_pub', String)
                rospy.loginfo("Resumed movement at green light")
            elif color.data == "Yellow":
                twist.linear.x = vmax/2
                twist.angular.z = 0
                pub.publish(twist)
                rospy.loginfo("Moving slowly at yellow light")
                while color.data != "Red":
                    color = rospy.wait_for_message('color_pub', String)
                rospy.loginfo("STOPPED at red light")
            else:
                rospy.loginfo("Continuing path with green light")

    except rospy.ROSInterruptException:
        pass
