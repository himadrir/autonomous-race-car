#!/usr/bin/env python3

import rospy
import math
import numpy as np
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header



class follow_gap:

    def __init__(self) -> None:

        self.drive_pub = rospy.Publisher("/fg_drive", AckermannDriveStamped, queue_size=10)
        self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.laser_callback)
        self.speed = 0.75
        self.min_angle = -60 / 180 * math.pi   #-70 to 70 to check for obstacles
        self.max_angle = 60/180 * math.pi
        self.header = Header()
        self.ranges = []
        self.radius = 9  #safety bubble to put on obstacle, basically width of the car
        self.min_idx = 0
        self.max_idx = 0
        self.angle = 0.0
        self.rate = rospy.Rate(15)
        self.lidar_min_angle = 0.0
        self.lidar_increment = 0.0


    def run(self):

        while not rospy.is_shutdown():

            if len(self.ranges) > 0:

                _closest_pt_dist, _closest_pt_idx = self.find_closest_point()
                _start, _end = self.find_max_gap(closest_pt_idx=_closest_pt_idx)
                self.best_point(start=_start, end=_end)
                self.controller()

            else:

                rospy.loginfo("NO lidar scan received!!")

            self.rate.sleep()


    def laser_callback(self, lidar_data):

        self.ranges = np.array(lidar_data.ranges)
        
        self.lidar_increment = lidar_data.angle_increment
        self.lidar_min_angle = lidar_data.angle_min

        self.min_idx = int(math.floor((self.min_angle - lidar_data.angle_min) / lidar_data.angle_increment))
        self.max_idx = int(math.ceil((self.max_angle - lidar_data.angle_min) / lidar_data.angle_increment))

        for i in range(self.min_idx, self.max_idx+1):

            if np.isnan(lidar_data.ranges[i]) or np.isinf(lidar_data.ranges[i]):
                self.ranges[i] = 0

            elif lidar_data.ranges[i] > lidar_data.range_max:
                self.ranges[i] = lidar_data.range_max

        #print(self.ranges)


    def find_closest_point(self):
        closest_pt_idx = self.min_idx
        closest_pt_dist = math.inf
        for i in range(self.min_idx, self.max_idx+1):
            d = sum( self.ranges[i-1:i+1] ) / 3  #avg values to eliminate noise
            if d < closest_pt_dist:
                closest_pt_dist = d
                closest_pt_idx = i
        
        return [closest_pt_dist, closest_pt_idx]
    

    def find_max_gap(self,closest_pt_idx):
        
        for i in range(closest_pt_idx - self.radius, closest_pt_idx + self.radius + 1):
            self.ranges[i] = 0

        start = self.min_idx
        end = self.min_idx
        curr_start = self.min_idx - 1
        curr_len = 0
        longest_len = 0

        for i in range(self.min_idx, self.max_idx + 1):

            if curr_start < self.min_idx:

                if self.ranges[i] > 0.0:
                    curr_start = i
                
            elif self.ranges[i] <= 0.0:
                curr_len = i - curr_start
                if longest_len < curr_len:
                    longest_len = curr_len
                    start = curr_start
                    end = i - 1
                curr_start = self.min_idx - 1


        if curr_start >= self.min_idx:
            curr_len = self.max_idx + 1 - curr_start
            if curr_len > longest_len:
                longest_len = curr_len
                start = curr_start
                end = self.max_idx

        return [start, end]


    def best_point(self, start, end):
        
        curr_max = 0

        for i in range(start, end + 1):

            if self.ranges[i] > curr_max:
                curr_max = self.ranges[i]
                #self.angle = lidar_data.angle_min + i * lidar_data.angle_increment
                self.angle = self.lidar_min_angle + i * self.lidar_increment
            
            elif self.ranges[i] == curr_max:
                if abs(self.lidar_min_angle + i * self.lidar_increment) < abs(self.angle):
                    #self.angle = lidar_data.angle_min + i * lidar_data.angle_increment
                    self.angle = self.lidar_min_angle + i * self.lidar_increment


    def controller(self):

        ack_msg = AckermannDriveStamped()
        self.header.stamp = rospy.Time.now()
        self.header.frame_id = "base_link"
        ack_msg.header = self.header
        ack_msg.drive.steering_angle = self.angle
        ack_msg.drive.speed = self.speed
        self.drive_pub.publish(ack_msg)

        #rospy.loginfo("steer angle: {} deg".format((self.angle*180)/math.pi))


if __name__ == '__main__':
    rospy.init_node('gap_follower_node')
    gg = follow_gap()
    gg.run()




