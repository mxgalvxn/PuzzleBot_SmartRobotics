#!/usr/bin/env python

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

def image_callback(msg):
    bridge = CvBridge()
    try:
        # Convert ROS Image message to OpenCV image
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        rospy.logerr(e)

    # Convert BGR image to HSV image
    hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    # Define color thresholds
    lower_red = (0, 100, 100)
    upper_red = (10, 255, 255)
    lower_yellow = (20, 100, 100)
    upper_yellow = (30, 255, 255)
    lower_green = (60, 100, 100)
    upper_green = (70, 255, 255)

    # Threshold the image to get the color regions
    mask_red = cv2.inRange(hsv_image, lower_red, upper_red)
    mask_yellow = cv2.inRange(hsv_image, lower_yellow, upper_yellow)
    mask_green = cv2.inRange(hsv_image, lower_green, upper_green)

    # Find contours in the color regions
    _, contours_red, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    _, contours_yellow, _ = cv2.findContours(mask_yellow, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    _, contours_green, _ = cv2.findContours(mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Draw contours and label the colors
    font = cv2.FONT_HERSHEY_SIMPLEX
    if len(contours_red) > 0:
        cv2.drawContours(cv_image, contours_red, -1, (0, 0, 255), 2)
        cv2.putText(cv_image, "Red", (10, 50), font, 2, (0, 0, 255), 2)
    elif len(contours_yellow) > 0:
        cv2.drawContours(cv_image, contours_yellow, -1, (0, 255, 255), 2)
        cv2.putText(cv_image, "Yellow", (10, 50), font, 2, (0, 255, 255), 2)
    elif len(contours_green) > 0:
        cv2.drawContours(cv_image, contours_green, -1, (0, 255, 0), 2)
        cv2.putText(cv_image, "Green", (10, 50), font, 2, (0, 255, 0), 2)
    else:
        cv2.putText(cv_image, "No color detected", (10, 50), font, 2, (255, 255, 255), 2)

    # Display the image
    cv2.imshow("Color detection", cv_image)
    cv2.waitKey(1)

    # Publish the image
    try:
        image_pub.publish(bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
        rospy.logerr(e)

if __name__ == '__main__':
    rospy.init_node('color_detection', anonymous=True)

    # Subscribe to the image topic
    image_sub =
if __name__ == '__main__':
    rospy.init_node('color_detection', anonymous=True)

    # Subscribe to the image topic
    image_sub = rospy.Subscriber("/camera/image_raw", Image, image_callback)

    # Publish the image topic
    image_pub = rospy.Publisher("/camera/color_detection", Image, queue_size=10)

    # OpenCV initialization
    cv2.namedWindow("Color detection", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Color detection", 640, 480)

    # Keep the node running
    rospy.spin()
