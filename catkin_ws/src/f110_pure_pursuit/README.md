# f110_pure_pursuit

An Implementation of pure pursuit tracking algorithm for f110 autonomous car proposed by the Robotics Institute, CMU.

This ROS node runs the pure pursuit algorithm that provides the tracking of robot given path and localization information (Pose). The node subscribes to a topic that publishes PoseStamped usually obtained using some form of localization. Make sure that the transforms are published between the frame of localization "base_link" and the reference map "map".

To test in a simulated environment, the racecar simulator can be cloned from [mLab's racecar_simulator repository](https://github.com/mlab-upenn/racecar_simulator) and the pure pursuit node can be launched using rosrun. (True Localized Pose is published in simulated environments on "gt_pose")

