#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <visualization_msgs/Marker.h>

#include "f110_pure_pursuit/csv_reader.h"
#include "f110_pure_pursuit/pure_pursuit.h"
#include "f110_pure_pursuit/types.h"


class PurePursuit
{
public:
    PurePursuit() :
        node_handle_(ros::NodeHandle()),
        pose_sub_(node_handle_.subscribe("gmcl_pose_stamped", 1, &PurePursuit::pose_callback, this)),
        drive_pub_(node_handle_.advertise<ackermann_msgs::AckermannDriveStamped>("nav", 1)),
        way_point_viz_pub_(node_handle_.advertise<visualization_msgs::Marker>("waypoint_markers", 100)),
        visualized_(false),
        last_best_index_(0),
        tf_listener_(tf_buffer_)
    {
        node_handle_.getParam("/lookahead_distance", lookahead_distance_);
        node_handle_.getParam("/high_speed", high_speed_);
        node_handle_.getParam("/low_speed", low_speed_);
        node_handle_.getParam("/n_way_points", n_way_points_);
        f110::CSVReader reader("/home/himadrir/catkin_ws/src/controller/src/coordinates.csv");
        way_point_data_ = reader.getData(n_way_points_);
        ROS_INFO("Pure Pursuit Node Initialized");
        visualize_waypoint_data();
        ROS_INFO("Way Points Published as Markers.");
        ros::Duration(1.0).sleep();
    }

    void add_way_point_visualization(const f110::WayPoint& way_point, const std::string& frame_id,
            double r, double g, double b, double transparency = 0.5, double scale_x=0.1, double scale_y=0.1,
            double scale_z=0.1)
    {
        visualization_msgs::Marker way_point_marker;
        way_point_marker.header.frame_id = frame_id;
        way_point_marker.header.stamp = ros::Time();
        way_point_marker.ns = "pure_pursuit";
        way_point_marker.id = unique_marker_id_;
        way_point_marker.type = visualization_msgs::Marker::SPHERE;
        way_point_marker.action = visualization_msgs::Marker::ADD;
        way_point_marker.pose.position.x = way_point.x;
        way_point_marker.pose.position.y = way_point.y;
        way_point_marker.pose.position.z = 0;
        way_point_marker.pose.orientation.x = 0.0;
        way_point_marker.pose.orientation.y = 0.0;
        way_point_marker.pose.orientation.z = 0.0;
        way_point_marker.pose.orientation.w = 1.0;
        way_point_marker.scale.x = scale_x;
        way_point_marker.scale.y = scale_y;
        way_point_marker.scale.z = scale_z;
        way_point_marker.color.a = transparency;
        way_point_marker.color.r = r;
        way_point_marker.color.g = g;
        way_point_marker.color.b = b;
        way_point_viz_pub_.publish(way_point_marker);
        unique_marker_id_++;
    }

    void visualize_waypoint_data()
    {
        const size_t increment = way_point_data_.size()/5;
        for(size_t i=0, j=0; i<way_point_data_.size(); i=i+increment, j++)
        {
            add_way_point_visualization(way_point_data_[i], "map", 0.0, 0.0, 1.0, 0.5);
        }
    }

    ///
    /// @param pose_msg - Localized Pose of the Robot
    void pose_callback(const geometry_msgs::PoseStamped::ConstPtr &pose_msg)
    {
        // Convert pose_msg to WayPoint
        const auto current_way_point = f110::WayPoint(pose_msg);

        // Transform Points
        const auto transformed_way_points = transform(way_point_data_, current_way_point, tf_buffer_, tf_listener_);

        // Find the best waypoint to track (at lookahead distance)
        const auto goal_way_point_index = f110::get_best_track_point_index(transformed_way_points, lookahead_distance_, last_best_index_);

        geometry_msgs::TransformStamped map_to_base_link;
        map_to_base_link = tf_buffer_.lookupTransform("base_footprint", "map", ros::Time(0));
        geometry_msgs::Pose goal_way_point;
        goal_way_point.position.x = way_point_data_[goal_way_point_index].x;
        goal_way_point.position.y = way_point_data_[goal_way_point_index].y;
        goal_way_point.position.z = 0;
        goal_way_point.orientation.x = 0;
        goal_way_point.orientation.y = 0;
        goal_way_point.orientation.z = 0;
        goal_way_point.orientation.w = 1;
        tf2::doTransform(goal_way_point, goal_way_point, map_to_base_link);

        add_way_point_visualization(goal_way_point, "base_footprint", 1.0, 0.0, 0.0, 0.3, 0.2, 0.2, 0.2);

        // Calculate curvature/steering angle
        const double steering_angle = 2*(goal_way_point.position.y)/(lookahead_distance_*lookahead_distance_);

        // Publish drive message
        ackermann_msgs::AckermannDriveStamped drive_msg;
        drive_msg.header.stamp = ros::Time::now();
        drive_msg.header.frame_id = "base_footprint";

        // Thresholding for limiting the movement of car wheels to avoid servo locking and variable speed
        // adjustment
        drive_msg.drive.steering_angle = steering_angle;
        if(steering_angle > 0.1)
        {
            if (steering_angle > 0.2)
            {
                drive_msg.drive.speed = low_speed_;
                if (steering_angle > 0.4)
                {
                    drive_msg.drive.steering_angle = 0.4;
                }
            }
            else
            {
                drive_msg.drive.speed = medium_speed_;
            }
        }
        else if(steering_angle < -0.1)
        {
            if (steering_angle < -0.2)
            {
                drive_msg.drive.speed = low_speed_;
                if (steering_angle < -0.4)
                {
                    drive_msg.drive.steering_angle = -0.4;
                }
            }
            else
            {
                drive_msg.drive.speed = medium_speed_;
            }
        }
        else
        {
            drive_msg.drive.speed = high_speed_;
        }
        drive_pub_.publish(drive_msg);
    }

private:
    ros::NodeHandle node_handle_;
    ros::Subscriber pose_sub_;
    ros::Publisher way_point_viz_pub_;
    ros::Publisher drive_pub_;

    double lookahead_distance_;
    double high_speed_;
    double medium_speed_;
    double low_speed_;
    int n_way_points_;

    std::vector<f110::WayPoint> way_point_data_;

    bool visualized_;
    size_t unique_marker_id_;
    size_t last_best_index_;
    tf2_ros::TransformListener tf_listener_;
    tf2_ros::Buffer tf_buffer_;
};

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "pure_pursuit_node");
    PurePursuit pp;
    ros::spin();
    return 0;
}
