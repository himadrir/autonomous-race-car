# Autonomous Race Car
checkout [wiki](https://github.com/himadrir/self-driving-car/wiki) for updates on progress.

## Dependencies 
- [ackermann msgs](http://wiki.ros.org/ackermann_msgs)
- [ld19 lidar ros driver](https://github.com/ldrobotSensorTeam/ldlidar_stl_ros)
- [vesc ros driver](https://github.com/mit-racecar/vesc)
- [hector slam](http://wiki.ros.org/hector_slam)
- [amcl](http://wiki.ros.org/amcl)
- [witmotion imu ros driver](https://github.com/ElettraSciComp/witmotion_IMU_ros)
- [rosserial driver](http://wiki.ros.org/rosserial)

## TF tree
map -> odom -> base_footprint -> base_link  

imu and laser connected to base_link
