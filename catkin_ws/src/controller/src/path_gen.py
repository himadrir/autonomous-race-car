#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import csv

def generate_path_from_csv(csv_file):
    path_msg = Path()
    path_msg.header.frame_id = 'map'  
    
    with open(csv_file, 'r') as file:
        reader = csv.reader(file)
        for row in reader:
            x = float(row[0])
            y = float(row[1])
            w = 0

            pose = PoseStamped()
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = "map"
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = w
            
            path_msg.poses.append(pose)

    return path_msg

if __name__ == '__main__':
    rospy.init_node('path_generator')
    
    csv_file = '/home/himadrir/catkin_ws/src/controller/src/coordinates.csv'
    
    path_msg = generate_path_from_csv(csv_file)
    
    # Publish the path
    path_pub = rospy.Publisher('pure_pursuit/path', Path, queue_size=10)
    
    rate = rospy.Rate(10)  # Publish rate (10 Hz)
    
    while not rospy.is_shutdown():
        path_pub.publish(path_msg)
        rate.sleep()
