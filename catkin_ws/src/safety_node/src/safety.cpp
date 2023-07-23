#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <std_msgs/Bool.h>


class safety_node
{
    private:
        ros::NodeHandle nh;
        double rs;
        double threshold_;
        ros::Publisher brake_flag_pub;
        ros::Publisher brake_pub;
        ros::Subscriber scan_sub;
        ros::Subscriber odom_sub;
        ros::Rate rate_;

    public:
        safety_node(): nh(ros::NodeHandle()), 
                       rs(0.0), 
                       brake_flag_pub(nh.advertise<std_msgs::Bool>("brake_flag", 1)),
                       brake_pub(nh.advertise<ackermann_msgs::AckermannDriveStamped>("brake", 1)),
                       scan_sub(nh.subscribe("/scan", 10, &safety_node::scan_callback, this)),
                       odom_sub(nh.subscribe("/odom", 10, &safety_node::odom_callback, this)),
                       rate_(60)
        {
            if(nh.getParam("braking_threshold", threshold_))
            {

            }
            
            else
            {
                std::cout<<"braking_threshold value not found, using default 0.10!";
            }
        }

        void scan_callback(const sensor_msgs::LaserScan::ConstPtr &scan_msg)
        {
            double min_ttc = std::numeric_limits<double>::max();
            const auto angle_increment = scan_msg->angle_increment;
            auto theta = scan_msg->angle_min;

            for(const auto& scan:scan_msg->ranges)
            {
                if(!std::isnan(scan) && !std::isinf(scan))
                {
                    const double current_ttc = scan / std::max(0.0, -rs * cos(theta));
                    if(current_ttc < min_ttc)
                    {
                        min_ttc = current_ttc;
                    }
                }
                theta = theta + angle_increment;
            }
            std::cout<<"time to collision: %f", min_ttc;
            std_msgs::Bool brake_flag;
            ackermann_msgs::AckermannDriveStamped ack_msg;

            if(min_ttc < threshold_)
            {   
                std::cout<<"EMERGENCY BRAKE ENABLED!!!";
                brake_flag.data = true;
                ack_msg.header = scan_msg->header;
                ack_msg.drive.speed = 0.0;

                brake_flag_pub.publish(brake_flag);
                brake_pub.publish(ack_msg);
            }
            
            else
            {
                brake_flag.data = false;
                brake_flag_pub.publish(brake_flag);
            }

            rate_.sleep();

        }

        void odom_callback(const nav_msgs::Odometry::ConstPtr &odom_msg)
        {
            rs = -odom_msg->twist.twist.linear.x;
        }
                       
};

int main(int argc, char ** argv) 
{
    ros::init(argc, argv, "safety");
    safety_node sn;
    ros::spin();
    return 0;
}