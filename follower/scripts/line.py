#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32

import cv2 as cv
import numpy as np
from cv_bridge import CvBridge, CvBridgeError

# Initialize the ROS Node named 'opencv_example', allow multiple nodes to be run with this name
rospy.init_node('lane_detector', anonymous=True)

# Print "Hello ROS!" to the Terminal and to a ROS Log file located in ~/.ros/log/loghash/*.log
#rospy.loginfo("Hello ROS!")

# Initialize the CvBridge class
bridge = CvBridge()

def get_line_minimums(grad):
    left_vec = grad[:50]
    center_vec = grad[51:429]
    right_vec = grad[430:]

    left_min_index = np.where(left_vec == np.amin(left_vec))[0][0]
    center_min_index = np.where(center_vec == np.amin(center_vec))[0][0] + 50
    right_min_index = np.where(right_vec == np.amin(right_vec))[0][0] + 430

    return left_min_index, right_min_index, center_min_index

def derivate_it(v_sum):
    derpoints = []
    for i in range(len(v_sum) - 1):
        if v_sum[i + 1] is not None:
            derpoints.append((v_sum[i + 1] - 2 * v_sum[i] + v_sum[i]) / (i - (i - 1)))
    derpoints.append(0)
    return derpoints

def lane_detector(img):
    # Preprocess image first
    img_resized = cv.resize(img, (480, 480))
    gray = cv.cvtColor(img_resized, cv.COLOR_BGR2GRAY)
    gray_blurred = cv.blur(gray, (13, 13)) # Image with blur

    # Getting roi
    roi_img = gray_blurred[430:480, 0:480]
    vertical_sum = roi_img.sum(axis=0)

    # Getting the center of the roi
    grad = derivate_it(vertical_sum)
    left_min, right_min, center_min = get_line_minimums(
        grad
    )

    # Calculating error
    error = 240 - center_min #240 reference pixel

    pub_vision_error.publish(Float32(error))

# Define a function to show the image in an OpenCV Window
def show_image(img):

    img = cv.flip(img, 0) #Gira horizontalmente

    # Detect lanes
    lane_detector(img)

# Define a callback for the Image message
def image_callback(img_msg):
    # log some info about the image topic
    rospy.loginfo(img_msg.header)

    # Try to convert the ROS Image message to a CV2 Image
    try:
        cv_image = bridge.imgmsg_to_cv2(img_msg, "passthrough")
    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {0}".format(e))

    # Flip the image 90deg
    #cv_image = cv2.transpose(cv_image)
    cv_image = cv.flip(cv_image,1)

    # Show the converted image
    show_image(cv_image)

# Initalize a subscriber to the "/camera/rgb/image_raw" topic with the function "image_callback" as a callback
sub_image = rospy.Subscriber("/video_source/raw", Image, image_callback)
pub_vision_error = rospy.Publisher("/lane_error", Float32, queue_size=10)

# Loop to keep the program from shutting down unless ROS is shut down, or CTRL+C is pressed
while not rospy.is_shutdown():
    rospy.spin()