#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Header, Int32, Bool
import math


class Joystick:
    def __init__(self):
        self.joy_sub = rospy.Subscriber("/joy", Joy, self.joy_callback)
        self.joy_pub = rospy.Publisher("/joy_drive", AckermannDriveStamped, queue_size=1)
        self.auto_control_pub = rospy.Publisher("/auto_flag", Int32, queue_size=1)
        self.enable_car_bool = rospy.Publisher("/en_car", Bool, queue_size=1)
        
        self.last_joy = None
        self.rate = rospy.Rate(40)
        self.header = Header()
        
        self.ackermann_msg = AckermannDriveStamped()
        self.auto_control_msg = Int32()
        self.throttle = 0
        self.steering_ang = 0
        self.enable_car_bool_msg = Bool()

    def joy_callback(self, msg):
        self.last_joy = msg

        if msg.buttons[1] == 1 and msg.axes[1] < 0:
            self.throttle = msg.axes[1] * 1
        else:
            self.throttle = msg.axes[1] * 1

        self.steering_ang = msg.axes[3] * math.pi/4

        self.enable_car_bool_msg.data = True if msg.buttons[4] and msg.buttons[5] else False
        
        self.auto_control_msg.data = 1 if msg.axes[5] < 0 else 0
        
        
    def run(self):
        while not rospy.is_shutdown():
            if self.last_joy:
                self.header.stamp = rospy.Time.now()
                self.header.frame_id = "base_link"
                
                self.ackermann_msg.header = self.header
                self.ackermann_msg.drive.speed = self.throttle
                self.ackermann_msg.drive.steering_angle = self.steering_ang
       
		
                self.joy_pub.publish(self.ackermann_msg)
                
                self.auto_control_pub.publish(self.auto_control_msg)
                self.enable_car_bool.publish(self.enable_car_bool_msg)
                
		 
            else:
                rospy.loginfo("No new msgs!")
            self.rate.sleep()


if __name__ == "__main__":
    rospy.init_node("car_command")
    jj = Joystick()
    jj.run()
