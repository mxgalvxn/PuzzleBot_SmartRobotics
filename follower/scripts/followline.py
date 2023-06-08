#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool, Int32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from geometry_msgs.msg import Twist

bridge = CvBridge()
image = None
line_position = None
previous_color = None

def image_callback(msg):
    global image
    image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    process_image()

def process_image():
    global image, line_position, previous_color
    if image is None:
        return

    # Preprocess the image (resize, convert to grayscale, etc.)
    resized_image = cv2.resize(image, (320, 240))
    hsv_image = cv2.cvtColor(resized_image, cv2.COLOR_BGR2HSV)

    # Define color thresholds
    green_lower = np.array([40, 50, 50])
    green_upper = np.array([80, 255, 255])
    yellow_lower = np.array([20, 100, 100])
    yellow_upper = np.array([40, 255, 255])
    red_lower1 = np.array([0, 100, 100])
    red_upper1 = np.array([10, 255, 255])
    red_lower2 = np.array([170, 100, 100])
    red_upper2 = np.array([180, 255, 255])

    # Create masks for color detection
    green_mask = cv2.inRange(hsv_image, green_lower, green_upper)
    yellow_mask = cv2.inRange(hsv_image, yellow_lower, yellow_upper)
    red_mask1 = cv2.inRange(hsv_image, red_lower1, red_upper1)
    red_mask2 = cv2.inRange(hsv_image, red_lower2, red_upper2)
    red_mask = cv2.bitwise_or(red_mask1, red_mask2)

    # Perform line detection and calculate line position
    line_position = detect_line(green_mask)

    # Publish line detection results
    if line_position is not None:
        line_detected_pub.publish(True)
        line_position_pub.publish(line_position)
    else:
        line_detected_pub.publish(False)
        line_position_pub.publish(0)

    # Detect color and take appropriate action
    color = detect_color(green_mask, yellow_mask, red_mask)

    if previous_color == "red" and color == "green":
        twist_cmd = Twist()
        if line_position is not None:
            twist_cmd.angular.z = np.clip(0.0037 * (160 - line_position), -0.3, 0.3)
        else:
            twist_cmd.angular.z = 0.0
        cmd_vel_pub.publish(twist_cmd)
        previous_color = color
    elif color == "green":
        twist_cmd = Twist()
        if line_position is not None:
            twist_cmd.angular.z = np.clip(0.0037 * (160 - line_position), -0.3, 0.3)
        else:
            twist_cmd.angular.z = 0.0
        cmd_vel_pub.publish(twist_cmd)
        previous_color = color
    elif color == "yellow":
        twist_cmd = Twist()
        if line_position is not None:
            twist_cmd.angular.z = np.clip(0.0037 * (160 - line_position), -0.3, 0.3)
        else:
            twist_cmd.angular.z = 0.0
        cmd_vel_pub.publish(twist_cmd)
        previous_color = color
    elif color == "red":
        twist_cmd = Twist()
        twist_cmd.linear.x = 0.0
        twist_cmd.angular.z = 0.0
        cmd_vel_pub.publish(twist_cmd)
        previous_color = color
    else:
        twist_cmd = Twist()
        twist_cmd.linear.x = 0.0
        twist_cmd.angular.z = 0.0
        cmd_vel_pub.publish(twist_cmd)

    # Publish filtered image
    filtered_image_msg = bridge.cv2_to_imgmsg(green_mask, encoding="mono8")
    filtered_image_pub.publish(filtered_image_msg)

def detect_line(image):
    col_sum = np.sum(image, axis=0)
    col_avg = np.average(col_sum)

    if col_avg < 400:
        return None

    line_position = np.mean(np.where(col_sum > 1000))

    return int(line_position)

def detect_color(green_mask, yellow_mask, red_mask):
    if np.any(green_mask):
        return "green"
    elif np.any(yellow_mask):
        return "yellow"
    elif np.any(red_mask):
        return "red"
    else:
        return None

def control_loop():
    global line_position, previous_color
    rate = rospy.Rate(20)  # Hz
    last_error = 0.0
    integral = 0.0
    Kp = 0.0037  # PID controller gains
    Ki = 0.0015
    Kd = 0.0

    while not rospy.is_shutdown():
        if line_position is not None:
            error = 160 - line_position
            integral += error
            derivative = error - last_error

            angular_velocity = Kp * error + Ki * integral + Kd * derivative

            twist_cmd = Twist()
            twist_cmd.linear.x = 0.1
            twist_cmd.angular.z = np.clip(angular_velocity, -0.3, 0.3)

            cmd_vel_pub.publish(twist_cmd)

            last_error = error

        rate.sleep()

def shutdown_callback():
    rospy.loginfo("Shutting down node...")
    twist_cmd = Twist()
    twist_cmd.linear.x = 0.0
    twist_cmd.angular.z = 0.0
    cmd_vel_pub.publish(twist_cmd)
    rospy.sleep(1)  # Esperar un segundo para asegurar que se publique el mensaje
    rospy.signal_shutdown("Node shutdown")

if __name__ == "__main__":
    rospy.init_node("line_follower")

    bridge = CvBridge()
    image_sub = rospy.Subscriber("/video_source/raw", Image, image_callback)
    line_detected_pub = rospy.Publisher("line_detected", Bool, queue_size=10)
    line_position_pub = rospy.Publisher("line_position", Int32, queue_size=10)
    cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
    filtered_image_pub = rospy.Publisher("filtered_image", Image, queue_size=10)
    rospy.on_shutdown(shutdown_callback)

    control_loop()