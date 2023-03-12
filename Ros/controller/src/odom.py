#!/usr/bin/env python3
import rospy
import math
import tf
from custom_msgs.msg import velocity_msg
from nav_msgs.msg import Odometry

vel_x, vel_y, vel_z = 0, 0, 0

yaw, x_, y_ = 0.0, 0.0, 0.0

curr_time, last_time = 0, 0


def vel_cb(velocity):
    global vel_x, vel_y, vel_z
    global x_, y_, yaw, curr_time, last_time, odom_pub, odom_broadcaster

    vel_x = velocity.vel_x
    vel_y = velocity.vel_y
    vel_z = velocity.vel_z

    curr_time = rospy.Time.now()
    dt = (curr_time - last_time).to_sec()

    last_time = curr_time

    delta_x = vel_x * math.cos(yaw) * dt
    delta_y = vel_x * math.sin(yaw) * dt
    delta_heading = vel_z * dt

    x_ += delta_x
    y_ += delta_y
    yaw += delta_heading

    odom_quat = tf.transformations.quaternion_from_euler(0, 0, yaw)
    odom_broadcaster.sendTransform((x_, y_, 0.0), odom_quat, curr_time, "base_footprint", "odom")

    odom_msg = Odometry()
    odom_msg.header.frame_id = "odom"
    odom_msg.child_frame_id = "base_footprint"
    odom_msg.header.stamp = curr_time

    odom_msg.pose.pose.position.x = x_
    odom_msg.pose.pose.position.y = y_
    odom_msg.pose.pose.position.z = 0.0

    odom_msg.pose.pose.orientation.x = 0.0
    odom_msg.pose.pose.orientation.y = 0.0
    odom_msg.pose.pose.orientation.z = math.sin(yaw/2.0)
    odom_msg.pose.pose.orientation.w = math.cos(yaw/2.0)

    odom_msg.pose.covariance[0] = 0.2
    odom_msg.pose.covariance[7] = 0.2
    odom_msg.pose.covariance[35] = 0.4

    odom_msg.twist.twist.linear.x = vel_x
    odom_msg.twist.twist.linear.y = 0.0
    odom_msg.twist.twist.angular.z = vel_z

    odom_pub.publish(odom_msg)


if __name__ == '__main__':
    rospy.init_node('raw_wheel_odom_pub', anonymous=True)
    curr_time = rospy.Time.now()
    last_time = rospy.Time.now()

    odom_pub = rospy.Publisher("/raw_wheel_odom_ackermann", Odometry, queue_size=10)
    odom_broadcaster = tf.TransformBroadcaster()
    rospy.Subscriber("/curr_vel", velocity_msg, vel_cb)

    rospy.spin()