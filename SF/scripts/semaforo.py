#!/usr/bin/env python

import rospy
from std_msgs.msg import Int8
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

def detect_traffic_light(frame):
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Definir rangos de color para deteccion
    red_lower = np.array([0, 100, 100])
    red_upper = np.array([10, 255, 255])
    yellow_lower = np.array([20, 100, 100])
    yellow_upper = np.array([30, 255, 255])
    green_lower = np.array([60, 100, 100])
    green_upper = np.array([80, 255, 255])

    # Deteccion del color rojo
    red_mask = cv2.inRange(hsv_frame, red_lower, red_upper)
    red_pixels = cv2.countNonZero(red_mask)

    # Deteccion del color amarillo
    yellow_mask = cv2.inRange(hsv_frame, yellow_lower, yellow_upper)
    yellow_pixels = cv2.countNonZero(yellow_mask)

    # Deteccion del color verde
    green_mask = cv2.inRange(hsv_frame, green_lower, green_upper)
    green_pixels = cv2.countNonZero(green_mask)

    if red_pixels >= 500:
        publisher.publish(2)
    elif yellow_pixels >= 500:
        publisher.publish(1)
    elif green_pixels >= 500:
        publisher.publish(0)
    else:
        publisher.publish(-1)

def image_callback(msg):
    try:
        bridge = CvBridge()
        frame = bridge.imgmsg_to_cv2(msg, "bgr8")
        detect_traffic_light(frame)
    except Exception as e:
        rospy.logerr(e)

def main():
    rospy.init_node('traffic_light_detector')
    global publisher
    publisher = rospy.Publisher('color_flag', Int8, queue_size=10)
    rospy.Subscriber('/video_source/raw', Image, image_callback)
    rospy.spin()

if __name__ == '__main__':
    main()

