#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <std_msgs/Bool.h>

// The class that handles emergency braking
class Safety
{
public:
    Safety() : nodehandle_(ros::NodeHandle()) ,
               relative_speed_(0.0),
               brake_pub_(nodehandle_.advertise<ackermann_msgs::AckermannDriveStamped>("brake", 100)),
               brake_bool_pub_(nodehandle_.advertise<std_msgs::Bool>("brake_bool", 100)),
               scan_sub_(nodehandle_.subscribe("scan", 100, &Safety::scan_callback, this)),
               odom_sub_(nodehandle_.subscribe("odom", 100, &Safety::odom_callback, this)),
               rate_(60)
    {
        nodehandle_.getParam("/emergency_braking_threshold", threshold_);
    }

    void odom_callback(const nav_msgs::Odometry::ConstPtr &odom_msg)
    {
        relative_speed_ = -odom_msg->twist.twist.linear.x;
    }

    void scan_callback(const sensor_msgs::LaserScan::ConstPtr &scan_msg)
    {
        double min_ttc = std::numeric_limits<double>::max();
        const auto angle_increment = scan_msg->angle_increment;
        auto theta = scan_msg->angle_min;

        for(const auto& scan:scan_msg->ranges)
        {
            if(!std::isinf(scan) && !std::isnan(scan))
            {
                const double current_ttc = scan / std::max(0.0, -relative_speed_ * cos(theta));
                if (current_ttc < min_ttc)
                {
                    min_ttc = current_ttc;
                }
            }
            theta += angle_increment;
        }

        std_msgs::Bool brake_bool;
        if(min_ttc < threshold_)
        {
            brake_bool.data = true;
            brake_bool_pub_.publish(brake_bool);

            ackermann_msgs::AckermannDriveStamped ackermann_msg;
            ackermann_msg.header = scan_msg->header;
            ackermann_msg.drive.speed = 0.0;
            brake_pub_.publish(ackermann_msg);
        }
        else
        {
            brake_bool.data = false;
            brake_bool_pub_.publish(brake_bool);
        }
        rate_.sleep();
    }

private:
    ros::NodeHandle nodehandle_;
    double relative_speed_;
    double threshold_;
    ros::Publisher brake_pub_;
    ros::Publisher brake_bool_pub_;
    ros::Subscriber scan_sub_;
    ros::Subscriber odom_sub_;
    ros::Rate rate_;

};
int main(int argc, char ** argv) {
    ros::init(argc, argv, "safety_node");
    Safety sn;
    ros::spin();
    return 0;
}