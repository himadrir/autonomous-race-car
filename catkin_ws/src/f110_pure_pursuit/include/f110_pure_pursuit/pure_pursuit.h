//
// Created by yash on 10/20/19.
//

#pragma once

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <vector>

namespace f110
{

std::vector<WayPoint> transform(const std::vector<WayPoint>& reference_way_points, const WayPoint& current_way_point,
        const tf2_ros::Buffer& tfBuffer, const tf2_ros::TransformListener& tf2_listener)
{
    geometry_msgs::TransformStamped map_to_base_link;
    map_to_base_link = tfBuffer.lookupTransform("base_footprint", "map", ros::Time(0));

    std::vector<WayPoint> transformed_way_points;
    for(const auto& reference_way_point: reference_way_points)
    {
        geometry_msgs::Pose map_way_point;
        map_way_point.position.x = reference_way_point.x;
        map_way_point.position.y = reference_way_point.y;
        map_way_point.position.z = 0;
        map_way_point.orientation.x = 0;
        map_way_point.orientation.y = 0;
        map_way_point.orientation.z = 0;
        map_way_point.orientation.w = 1;

        tf2::doTransform(map_way_point, map_way_point, map_to_base_link);

        transformed_way_points.emplace_back(map_way_point);
    }
    return transformed_way_points;
}


size_t get_best_track_point_index(const std::vector<WayPoint>& way_point_data, double lookahead_distance, size_t& last_best_index)
{
    double closest_distance = std::numeric_limits<double>::max();
    const size_t way_point_size = way_point_data.size();

    auto update_best_index_within_interval = [&](const size_t start_index, const size_t end_index){
        for(size_t i=start_index; i <end_index; ++i)
        {
            if(way_point_data[i].x < 0) continue;
            double distance = sqrt(way_point_data[i].x*way_point_data[i].x + way_point_data[i].y*way_point_data[i].y);
            double lookahead_diff = std::abs(distance - lookahead_distance);
            if(lookahead_diff < closest_distance)
            {
                closest_distance = lookahead_diff;
                last_best_index = i;
            }
        }
    };

    if(last_best_index > way_point_size - way_point_size/10)
    {
        update_best_index_within_interval(way_point_size - way_point_size/10, way_point_size);
        update_best_index_within_interval(0, 100);
    }
    else
    {
        update_best_index_within_interval(last_best_index, std::min(last_best_index + 100, way_point_size));
    }
    ROS_DEBUG("closest_way_point_index %i", static_cast<int>(last_best_index));
    return last_best_index;
}

} // namespace f110

