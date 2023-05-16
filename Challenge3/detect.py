#!/usr/bin/env python3
# Description:
# - Subscribes to real-time streaming video from your built-in webcam.
#
# Author:
# - Addison Sears-Collins
# - https://automaticaddison.com
 
# Import the necessary libraries
import rospy # Python library for ROS
from sensor_msgs.msg import Image # Image is the message type
from std_msgs.msg import String # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
import ros_numpy
import numpy as np


def detect_color(hsv_img):
    # Define color ranges
    yellow_lower = np.array([20, 100, 100])
    yellow_upper = np.array([40, 255, 255])
   
    green_lower = np.array([40, 100, 100])
    green_upper = np.array([80, 255, 255])
   
    red_lower1 = np.array([0, 100, 100])
    red_upper1 = np.array([10, 255, 255])
    red_lower2 = np.array([170, 100, 100])
    red_upper2 = np.array([180, 255, 255])
   
    # Create masks for each color
    yellow_mask = cv2.inRange(hsv_img, yellow_lower, yellow_upper)
    green_mask = cv2.inRange(hsv_img, green_lower, green_upper)
    red_mask1 = cv2.inRange(hsv_img, red_lower1, red_upper1)
    red_mask2 = cv2.inRange(hsv_img, red_lower2, red_upper2)
    red_mask = cv2.bitwise_or(red_mask1, red_mask2)
   
    # Check if any color is detected
    if cv2.countNonZero(yellow_mask) > 0:
        return "Amarillo"
    elif cv2.countNonZero(green_mask) > 0:
        return "Verde"
    elif cv2.countNonZero(red_mask) > 0:
        return "Rojo"
    else:
        return "Ninguno"

def callback(data):
 
    # Used to convert between ROS and OpenCV images
    #br = CvBridge()
 
    # Output debugging information to the terminal
    #rospy.loginfo("Receiving video frame")
   
    # Convert ROS Image message to OpenCV image
    #current_frame = br.imgmsg_to_cv2(data)
   
    # Convert the OpenCV image to NumPy array
    current_frame = ros_numpy.numpify(data)

    # Convert BGR to HSV
    hsv_img = cv2.cvtColor(current_frame, cv2.COLOR_BGR2HSV)
   
    # Detect color
    color = detect_color(hsv_img)

    # Display image
    cv2.imshow("Camera", current_frame)
   
    # Print detected color
    #print("Color detectado:", color)
    pubcolor = rospy.Publisher('colorSignal', String, queue_size = 10)
    pubcolor.publish(color)
    cv2.waitKey(1)
     
def receive_message():
 
    # Tells rospy the name of the node.
    # Anonymous = True makes sure the node has a unique name. Random
    # numbers are added to the end of the name.
    rospy.init_node('video_color_py', anonymous=True)
   
    # Node is subscribing to the video_frames topic
    rospy.Subscriber('video_frames', Image, callback)
 
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
 
    # Close down the video stream when done
    cv2.destroyAllWindows()
 
if _name_ == '_main_':
    receive_message()
