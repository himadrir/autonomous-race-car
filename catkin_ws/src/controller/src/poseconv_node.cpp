#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>

ros::Publisher pub; 

void poseCallback(const geometry_msgs::PoseWithCovarianceStamped& input) {

  geometry_msgs::PoseStamped output;
  output.header = input.header;
  output.pose = input.pose.pose;
  pub.publish(output);
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "poseconv_node");

  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("gmcl_pose", 1, poseCallback);
  pub = nh.advertise<geometry_msgs::PoseStamped>("gmcl_pose_stamped", 1);

  ros::spin();

  return 0;
}