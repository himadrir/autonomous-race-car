import rospy
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import LaserScan
from custom_msgs.msg import velocity_msg
import numpy as np
from math import floor, atan, cos, sin

a_index = 0
prev_time, prev_error, integral = 0, 0, 0
laser_prev_time = 0
cmd_vel_pub = None
car_vel = 0.0

Kp = 1.00
Ki = 0.005
Kd = 0.001
distance_to_maintain = 0.5


def vel_callback(curr_vel):
    global car_vel
    car_vel = curr_vel.vel_x


def laser_callback(lidar_msg):
    global a_index, laser_prev_time

    a_, b_ = 0, 0

    laser_t_now = rospy.Time.now().to_sec()
    dt_laser = laser_t_now - laser_prev_time

    b_index = abs(floor((1.5708 - lidar_msg.angle_min) / lidar_msg.angle_increment))
    b_angle = 1.5708  # 90 deg
    a_angle = 0.785398  # 45 deg

    if lidar_msg.angle_min > 0.785398:
        a_angle = lidar_msg.angle_min
        a_index = 0
    else:
        a_index = abs(floor((0.785398 - lidar_msg.angle_min) / lidar_msg.angle_increment))

    if not np.isinf(lidar_msg.ranges[a_index]) and not np.isnan(lidar_msg.ranges[a_index]):
        a_ = lidar_msg.ranges[a_index]
    else:
        a_ = 100

    if not np.isinf(lidar_msg.ranges[b_index]) and not np.isnan(lidar_msg.ranges[b_index]):
        b_ = lidar_msg.ranges[b_index]
    else:
        b_ = 100

    alpha = atan((a_ * cos(b_angle - a_angle) - b_) / (a_ * sin(b_angle - a_angle)))
    d = b_ * cos(alpha)
    # d_ = d + 1.00 * sin(alpha) #replace 1 with curr velocity read from the topic /curr_vel * dt for distance
    d_ = d + (car_vel * dt_laser) * sin(alpha)
    error_ = distance_to_maintain - d_

    pid_controller(error_)

    laser_prev_time = laser_t_now


def pid_controller(error):
    global prev_time, integral, prev_error, cmd_vel_pub, Kp, Ki, Kd

    t_now = rospy.Time.now().to_sec()
    dt = t_now - prev_time
    theta = Kp * error + Kd * (error - prev_error) / dt

    vel_msg = TwistStamped()

    vel_msg.header.stamp = rospy.Time.now()
    vel_msg.header.frame_id = "racer"
    vel_msg.twist.angular.z = theta
    vel_msg.twist.linear.x = 0.3

    # @todo: adaptive speed for the car

    cmd_vel_pub.publish(vel_msg)

    prev_time = t_now
    prev_error = error


if __name__ == '__main__':
    try:
        rospy.init_node('wall_follow_node', anonymous=True)
        cmd_vel_pub = rospy.Publisher("/cmd_vel_", TwistStamped, queue_size=1)
        rospy.Subscriber("/scan", LaserScan, laser_callback)
        rospy.Subscriber("/curr_vel", velocity_msg, vel_callback)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
