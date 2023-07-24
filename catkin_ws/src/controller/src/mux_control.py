#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32, Header, Bool
from ackermann_msgs.msg import AckermannDriveStamped



class Mux:

    def __init__(self):

        self.joy_drive_sub = rospy.Subscriber("/joy_drive", AckermannDriveStamped, self.joy_drive_cb)
        self.auto_flag_sub = rospy.Subscriber("/auto_flag", Int32, self.auto_flag_cb)
        self.wf_drive_sub = rospy.Subscriber("/wf_drive", AckermannDriveStamped, self.wf_drive_cb)
        self.drive_pub = rospy.Publisher("/ackermann_cmd", AckermannDriveStamped, queue_size=1)
        self.brake_flag_sub = rospy.Subscriber("/brake_bool", Bool, self.brake_flag_cb)
        self.en_car_bool_sub = rospy.Subscriber("/en_car", Bool, self.en_car_bool_cb)

        self.rate = rospy.Rate(15)
        self.wf_ackermann_msg = AckermannDriveStamped()
        self.joy_ackermann_msg = AckermannDriveStamped()
        self.brake_msg_ = AckermannDriveStamped()
        self.header = Header()
        self.auto_flag = 0
        self.brake_flag = 0
        self.en_car_status = 0
        self.status_flag = False
        self.en_print_flag = "ENABLED"

    def auto_flag_cb(self, wf_flag_msg):
        if wf_flag_msg.data == 1:
            self.auto_flag = 1
        else:
            self.auto_flag = 0

    def wf_drive_cb(self, wf_msg):
        self.wf_ackermann_msg.drive = wf_msg.drive

    def joy_drive_cb(self, joy_cmd_msg):
        self.joy_ackermann_msg.drive = joy_cmd_msg.drive

    def brake_flag_cb(self, brake_flag_msg):
        self.brake_flag = brake_flag_msg.data

    def en_car_bool_cb(self, en_car_msg):
        if en_car_msg.data:
            self.en_car_status += 1

    def run(self):

        while not rospy.is_shutdown():

            self.header.stamp = rospy.Time.now()
            self.header.frame_id = "base_link"
            self.wf_ackermann_msg.header = self.header
            self.joy_ackermann_msg.header = self.header
            self.brake_msg_.header = self.header

            if self.en_car_status > 0:  #activate car
                if self.en_print_flag == "DISABLED":
                    rospy.loginfo("CAR ENABLED!")
                    self.en_print_flag = "ENABLED"

                if self.wf_ackermann_msg and self.joy_ackermann_msg:
                    
                    if not self.brake_flag:
                        
                        if self.auto_flag == 1:  #right trigger to change modes between auto and manual
                            if not self.status_flag:
                                rospy.loginfo("AUTO MODE")
                                self.status_flag = True
                            self.drive_pub.publish(self.wf_ackermann_msg)
                            
                        else:
                            if self.status_flag:
                                rospy.loginfo("MANUAL MODE")
                                self.status_flag = False
                            self.drive_pub.publish(self.joy_ackermann_msg)

                    else:
                        rospy.loginfo("EMERGENCY BRAKES ENABLED!!")
                        self.brake_msg_.drive.speed = 0.0
                        self.brake_msg_.drive.steering_angle = 0.0
                        self.drive_pub.publish(self.brake_msg_)
                        self.en_car_status = 0   #deactivate car after ebrake, to enable hold left and right button
                        

            elif not self.en_car_status and self.en_print_flag == "ENABLED":
                #print(self.en_print_flag)
                rospy.loginfo("CAR DISABLED!")
                self.en_print_flag = "DISABLED"



            self.rate.sleep()


if __name__ == "__main__":
    rospy.init_node("mux_control")
    mc = Mux()
    mc.run()

