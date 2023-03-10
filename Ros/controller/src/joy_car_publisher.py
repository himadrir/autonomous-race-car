#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Header
import math


class Joystick:
    def __init__(self):
        self.joy_sub = rospy.Subscriber("/joy", Joy, self.joy_callback)
        self.joy_pub = rospy.Publisher("/cmd_vel_", TwistStamped, queue_size=1)
        self.last_joy = None
        self.rate = rospy.Rate(20)
        self.header = Header()
        self.twist_msg = TwistStamped()
        self.throttle = 0
        self.steering_ang = 0

    def joy_callback(self, msg):
        self.last_joy = msg
        if msg.buttons[1] == 1 and msg.axes[1] < 0:
            self.throttle = msg.axes[1] * 1
        else:
            self.throttle = msg.axes[1] * 1

        self.steering_ang = msg.axes[2] * math.pi/4

    def run(self):
        while not rospy.is_shutdown():
            if self.last_joy:
                self.header.stamp = rospy.Time.now()
                self.header.frame_id = "base_link"
                self.twist_msg.header = self.header
                self.twist_msg.twist.linear.x = self.throttle
                self.twist_msg.twist.angular.z = self.steering_ang

                self.joy_pub.publish(self.twist_msg)

            else:
                rospy.loginfo("No new msgs!")
            self.rate.sleep()


if __name__ == "__main__":
    rospy.init_node("car_command")
    jj = Joystick()
    jj.run()
