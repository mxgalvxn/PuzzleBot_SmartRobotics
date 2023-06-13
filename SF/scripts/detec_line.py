#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from std_msgs.msg import Header

class LineFollower():
    def __init__(self):
        rospy.on_shutdown(self.shutdown)
        rate = rospy.Rate(100)
        self.cv_bridge = CvBridge()
        #self.image_sub = rospy.Subscriber('/video_source/raw', Image, self.image_callback)
        self.image_sub = rospy.Subscriber('/camera/image_raw', Image, self.image_callback)

        self.robot_vel_pub = rospy.Publisher('/robot_vel', Twist, queue_size=1)
        self.filtered_image_pub = rospy.Publisher('/filtered_image', Image, queue_size=1)
        twist_msg = Twist()

        # self.x = 350  # Coordenada x del borde izquierdo
        # self.y = 600  # Coordenada y del borde superior
        # self.width = 520
        # self.height = 400
        
        self.x = 350  # Coordenada x del borde izquierdo
        self.y = 600  # Coordenada y del borde superior
        self.width = 520
        self.height = 400

        self.linear_vel = 0.08    # Velocidad lineal del robot

        # Controlador PID
        u_w = [0.0, 0.0]
        e_w = [0.0, 0.0, 0.0]

        # kp_w = 0.005
        # ki_w = 0.0005
        # kd_w = 0.00005

        kp_w = 0.005
        ki_w = 0.0005
        kd_w = 0.00005
        self.w_max = .7


        delta_t = 1.0 / 50.0

        self.image_received_flag = 0

        K1_w = kp_w + delta_t * ki_w + kd_w / delta_t
        K2_w = - kp_w - 2.0 * kd_w / delta_t
        K3_w = kd_w / delta_t

        kernel = np.ones((5, 5), np.uint8)

        while not rospy.is_shutdown():
            if self.image_received_flag:
                self.image_received_flag = 0
                cv_image = self.frame

                roi = cv_image[self.y:self.y + self.height, self.x:self.x + self.width]

                roi = cv2.GaussianBlur(roi, (15, 15), 0)
                eroded = cv2.erode(roi, kernel, iterations=1)
                dilated = cv2.erode(eroded, kernel, iterations=1)

                gray_roi = cv2.cvtColor(dilated, cv2.COLOR_BGR2GRAY)

                _, binary_roi = cv2.threshold(gray_roi, 100, 125, cv2.THRESH_BINARY_INV)

                # Crear el mensaje de la imagen filtrada
                filtered_msg = Image()
                filtered_msg.header = Header()
                filtered_msg.header.stamp = rospy.Time.now()
                filtered_msg.header.frame_id = 'filtered_image'
                filtered_msg.encoding = 'mono8'
                filtered_msg.height, filtered_msg.width = binary_roi.shape
                filtered_msg.step = filtered_msg.width
                filtered_msg.data = binary_roi.flatten().tolist()

                # Publicar la imagen filtrada
                self.filtered_image_pub.publish(filtered_msg)

                _, contours, _ = cv2.findContours(binary_roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

                # Si se encuentran contornos

                if contours :	
                    line_contour = max(contours, key=cv2.contourArea)

                    cv2.drawContours(roi, [line_contour], -1, (0, 0, 255), thickness=3)

                    # Calcular el centro del contorno
                    M = cv2.moments(line_contour)
                    if M['m00'] > 0:
                        cx = int(M['m10'] / M['m00'])
                        cy = int(M['m01'] / M['m00'])
                        cv2.circle(roi, (cx, cy), 5, (0, 255, 0), -1)

                        # Controlador PID
                        e_w[0] = 260 - cx
                        u_w[0] = K1_w * e_w[0] + K2_w * e_w[1] + K3_w * e_w[2] + u_w[1]
                        e_w[2] = e_w[1]
                        e_w[1] = e_w[0]
                        u_w[1] = u_w[0]

                        if u_w [0] > self.w_max:
                            u_w[0] = self.w_max
                        elif u_w[0] < -self.w_max:
                            u_w[0] = -self.w_max

                        # Control de movimiento
                        angular_z = u_w[0]
                        linear_x = self.linear_vel

                        # Publicar los comandos de velocidad
                        twist_msg.linear.x = linear_x
                        twist_msg.angular.z = angular_z
                        #print(twist_msg)                        
                        self.robot_vel_pub.publish(twist_msg)

                        print("lineal",twist_msg.linear.x)
                        print("angular",twist_msg.angular.z)

            cv2.waitKey(1)
            rate.sleep()

    def image_callback(self, msg):
        try:
            self.frame = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self.image_received_flag = 1
        except CvBridgeError as e:
            print(e)

    def shutdown(self):
        zero = Twist()
        self.robot_vel_pub.publish(zero)


if __name__ == "__main__" :
    print("Initialized")
    rospy.init_node("detec_line", anonymous=True)
    LineFollower()