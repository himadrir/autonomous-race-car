#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32, Header
from ackermann_msgs.msg import AckermannDriveStamped



class Mux:

    def __init__(self):

        self.joy_drive_sub = rospy.Subscriber("/joy_drive", AckermannDriveStamped, self.joy_drive_cb)
        self.auto_flag_sub = rospy.Subscriber("/auto_flag", Int32, self.auto_flag_cb)
        self.wf_drive_sub = rospy.Subscriber("/wf_drive", AckermannDriveStamped, self.wf_drive_cb)
        self.drive_pub = rospy.Publisher("/drive", AckermannDriveStamped, queue_size=1)  #main publisher to robot

        self.rate = rospy.Rate(20)
        self.wf_ackermann_msg = AckermannDriveStamped()
        self.joy_ackermann_msg = AckermannDriveStamped()
        self.header = Header()
        self.auto_flag = 0

    def auto_flag_cb(self, wf_flag_msg):
        if wf_flag_msg.data == 1:
            self.auto_flag = not self.auto_flag

    def wf_drive_cb(self, wf_msg):
        self.wf_ackermann_msg.drive = wf_msg.drive

    def joy_drive_cb(self, joy_cmd_msg):
        self.joy_ackermann_msg.drive = joy_cmd_msg.drive

    def run(self):

        while not rospy.is_shutdown():

            if self.wf_ackermann_msg and self.joy_ackermann_msg:

                self.header.stamp = rospy.Time.now()
                self.header.frame_id = "base_link"
                self.wf_ackermann_msg.header = self.header
                self.joy_ackermann_msg.header = self.header

                if self.auto_flag == 1:
                    rospy.loginfo("WALL FOLLOW MODE")
                    self.drive_pub.publish(self.wf_ackermann_msg)
                else:
                    rospy.loginfo("MANUAL MODE")
                    self.drive_pub.publish(self.joy_ackermann_msg)

            self.rate.sleep()


if __name__ == "__main__":
    rospy.init_node("mux_control")
    mc = Mux()
    mc.run()
