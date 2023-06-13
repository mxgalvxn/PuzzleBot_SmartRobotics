#!/usr/bin/env python
from logging import shutdown
import rospy
from std_msgs.msg import Int8
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class Logic_Controller() :
    def __init__(self) :
        rospy.on_shutdown(self.shutdown)
        node_rate = 50
        rate = rospy.Rate(node_rate)

        rospy.Subscriber("color_flag", Int8, self.color_flag_cb)
        rospy.Subscriber("is_circle", Int8, self.is_circle_cb)
        rospy.Subscriber("robot_vel", Twist, self.robot_vel_cb)

        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

        self.robot_vel = Twist()
        self.color_flag = -1
        self.is_circle = 0
        self.radius = 0

        self.cmd_vel = Twist()
        self.zero = Twist()

        while not rospy.is_shutdown() :
            if self.color_flag == 2:
                #print("STOP")
                self.cmd_vel.linear.x = self.zero.linear.x
                self.cmd_vel.angular.z = self.zero.angular.z
            elif self.color_flag == 1:
                #print("WARNING")
                self.cmd_vel.linear.x = self.robot_vel.linear.x / 2
                self.cmd_vel.angular.z = self.robot_vel.angular.z / 2
            elif self.color_flag == 0:
                #print("CONTINUE")
                self.cmd_vel.linear.x = self.robot_vel.linear.x
                self.cmd_vel.angular.z = self.robot_vel.angular.z
            elif self.color_flag == -1:
                #print("MOVING WITHOUT RESTRICTIONS")
                self.cmd_vel.linear.x = self.robot_vel.linear.x
                self.cmd_vel.angular.z = self.robot_vel.angular.z
            else:
                self.cmd_vel.linear.x = 0
                self.cmd_vel.angular.z = 0

            self.vel_pub.publish(self.cmd_vel)
            rate.sleep()

    def color_flag_cb(self, color_flag) :
        self.color_flag = color_flag.data

    def is_circle_cb(self, is_circle) :
        self.is_circle = is_circle.data

    def robot_vel_cb(self, robot_vel) :
        self.robot_vel = robot_vel

    def shutdown(self) :
        self.vel_pub.publish(self.zero)
        print("LOGIC CONTROLLER NODE KILLED")

if __name__ == "__main__" :
    rospy.init_node("controller", anonymous = True)
    Logic_Controller()
