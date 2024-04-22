#include "f110_reactive_methods/utility.h"

#include <ackermann_msgs/AckermannDriveStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

/// Class that contains the ros utility and functions that implement the follow the gap method
class follow_the_gap
{
public:
    follow_the_gap() :
            node_handle_(ros::NodeHandle()),
            lidar_sub_(node_handle_.subscribe("scan", 100, &follow_the_gap::scan_callback, this)),
            drive_pub_(node_handle_.advertise<ackermann_msgs::AckermannDriveStamped>("nav", 100)),
            truncated_(false)
    {
        node_handle_.getParam("/bubble_radius", bubble_radius_);
        node_handle_.getParam("/smoothing_filter_size", smoothing_filter_size_);
        node_handle_.getParam("/truncated_coverage_angle", truncated_coverage_angle_);
        node_handle_.getParam("/velocity", velocity_);
        node_handle_.getParam("/max_accepted_distance", max_accepted_distance_);
        node_handle_.getParam("/error_based_velocities", error_based_velocities_);
        node_handle_.getParam("/steering_angle_reactivity", steering_angle_reactivity_);
    }

    /// PreProcess Lidar Scan to replace nans by zeros and find the closest point in the lidar scan
    /// @param scan_msg - Lidar Scan Msg Callback
    /// @param filtered_ranges - Filtered Ranges from the scan_msg
    /// @param closest_index - Index of the Closest Point
    /// @return The closest point in the lidar scan
    std::vector<double> preprocess_lidar_scan(const sensor_msgs::LaserScan::ConstPtr &scan_msg) const
    {
        std::vector<double> filtered_ranges;
        for(size_t i=truncated_start_index_; i<truncated_end_index_; ++i)
        {
            if(std::isnan(scan_msg->ranges[i]))
            {
                filtered_ranges.push_back(0.0);
            }
            else if(scan_msg->ranges[i] > max_accepted_distance_ || std::isinf(scan_msg->ranges[i]))
            {
                filtered_ranges.push_back(max_accepted_distance_);
            }
            else
            {
                filtered_ranges.push_back(scan_msg->ranges[i]);
            }
        }
        return fgm::apply_smoothing_filter(filtered_ranges, smoothing_filter_size_);
    }

    /// Get the Best Point in a Range of Max-Gap (according to some metric (In this case farthest point))
    /// @param filtered_ranges vector of max gap ranges
    /// @param start_index start index of max_gap_ranges in the filtered_ranges
    /// @param end_index end index of max_gap_ranges in the filtered_ranges
    /// @return return index of the Max Element in the Vector
    size_t get_best_point(const std::vector<double>& filtered_ranges, int start_index, int end_index) const
    {
        return (start_index + end_index)/2;
    }

    /// Calculates the required steering angle based on the angles of the best point and the current heading of vehicle
    /// @param scan_msg - Scan Msg received by the callback
    /// @param best_point_index - The best point for heading as per FGM algorithm
    /// @return required steering angle
    double get_steering_angle_from_range_index(const sensor_msgs::LaserScan::ConstPtr &scan_msg,
                                               const size_t best_point_index,
                                               const double closest_value)
    {
        const size_t best_point_index_input_scan_frame = truncated_start_index_ + best_point_index;
        double best_point_steering_angle;
        // Case When Steering Angle is to be negative
        if(best_point_index_input_scan_frame < scan_msg->ranges.size()/2)
        {
            best_point_steering_angle = - scan_msg->angle_increment*
                               static_cast<double>(scan_msg->ranges.size()/2.0 - best_point_index_input_scan_frame);
        }
            // Case When Steering Angle is to be negative
        else
        {
            best_point_steering_angle = scan_msg->angle_increment*
                               static_cast<double>(best_point_index_input_scan_frame - scan_msg->ranges.size()/2.0);
        }
        ROS_DEBUG("closest_value %f" , closest_value);
        const auto distance_compensated_steering_angle =
                std::clamp((best_point_steering_angle*steering_angle_reactivity_)/
                static_cast<double>(closest_value), -1.57, 1.57);
        return distance_compensated_steering_angle;
    }

    /// Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
    /// @param scan_msg
    void scan_callback(const sensor_msgs::LaserScan::ConstPtr &scan_msg)
    {
        if(!truncated_)
        {
            ROS_INFO("input scan start angle = %f", scan_msg->angle_min);
            ROS_INFO("input scan end angle = %f", scan_msg->angle_max);
            ROS_INFO("input scan start index = %u", 0);
            ROS_INFO("input scan end index = %u", static_cast<u_int>(scan_msg->ranges.size()));

            const auto truncated_indices =
                    fgm::truncated_start_and_end_indices(scan_msg, truncated_coverage_angle_);
            truncated_start_index_ = truncated_indices.first;
            truncated_end_index_ = truncated_indices.second;
            truncated_ = true;

            ROS_INFO("truncated scan start angle = %f", -truncated_coverage_angle_/2);
            ROS_INFO("truncated scan end angle = %f", truncated_coverage_angle_/2);
            ROS_INFO("truncated start index = %u", static_cast<u_int>(truncated_start_index_));
            ROS_INFO("truncated end index = %u", static_cast<u_int>(truncated_end_index_));
        }

        // Pre-Process (zero out nans and Filter)
        auto filtered_ranges = preprocess_lidar_scan(scan_msg);
        ROS_DEBUG("Created Filtered Ranges");

        // find the closest point to LiDAR
        const size_t closest_index = fgm::minimum_element_index(filtered_ranges);
        ROS_DEBUG("Closest Point Index = %u", static_cast<u_int>(closest_index + truncated_start_index_));

        const auto closest_range = filtered_ranges[closest_index];
        ROS_DEBUG("Closest Point Value = %f", filtered_ranges[closest_index]);

        // Eliminate all points inside 'bubble' (set them to zero)
        fgm::zero_out_safety_bubble(&filtered_ranges, closest_index, bubble_radius_);
        ROS_DEBUG("Zeroed out the Bubble Region");

        // Find max length gap
        const auto [start_index, end_index] = fgm::find_largest_nonzero_sequence(filtered_ranges);
        ROS_DEBUG("Max Gap Start Index = %u", static_cast<u_int>(start_index + truncated_start_index_));
        ROS_DEBUG("Max Gap End Index = %u", static_cast<u_int>(end_index + truncated_start_index_));

        // Find the best point in the gap
        const size_t best_point_index = get_best_point(filtered_ranges, start_index, end_index);
        ROS_DEBUG("Best Point = %u", static_cast<u_int>(best_point_index + truncated_start_index_ ));

        const double steering_angle = get_steering_angle_from_range_index(scan_msg, best_point_index, closest_range);
        ROS_DEBUG("Publish Steering Angle = %f", steering_angle);

        // Publish Drive message
        ackermann_msgs::AckermannDriveStamped drive_msg;
        drive_msg.header.stamp = ros::Time::now();
        drive_msg.header.frame_id = "laser";
        drive_msg.drive.steering_angle = steering_angle;
        if(abs(steering_angle) > 0.349)
        {
            drive_msg.drive.speed = error_based_velocities_["high"];
        }
        else if(abs(steering_angle) > 0.174)
        {
            drive_msg.drive.speed = error_based_velocities_["medium"];
        }
        else
        {
            drive_msg.drive.speed = error_based_velocities_["low"];
        }
        drive_pub_.publish(drive_msg);
    }

private:
    ros::NodeHandle node_handle_;
    ros::Subscriber lidar_sub_;
    ros::Publisher drive_pub_;

    double bubble_radius_;
    double max_accepted_distance_;
    double smoothing_filter_size_;
    double steering_angle_reactivity_;

    bool truncated_;
    double truncated_coverage_angle_;
    size_t truncated_start_index_;
    size_t truncated_end_index_;

    double velocity_;

    std::map<std::string, double> error_based_velocities_;
};

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "follow_the_gap_node");
    follow_the_gap gap_follower;
    ros::spin();
    return 0;
}