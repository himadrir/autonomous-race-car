//
// This header file contains helper functions for the follow_the_gap ros node
// Created by yash on 9/22/19.
//

#pragma once

#include <sensor_msgs/LaserScan.h>
#include <ros/ros.h>

#include <vector>

namespace fgm
{

/// Function to Apply 1d Smoothing Filter (Averaging Filter)
/// @param input_vector - Input Vector
/// @param smoothing_filter_size - Size of the averaging required
/// @return smoothened vector
std::vector<double> apply_smoothing_filter(const std::vector<double>& input_vector, size_t smoothing_filter_size)
{
    std::vector<double> smoothened_vector;
    for(size_t i=smoothing_filter_size; i<input_vector.size()-smoothing_filter_size; ++i)
    {
        double current_sum = 0;
        for(size_t j = i-smoothing_filter_size + 1; j<i+smoothing_filter_size; ++j)
        {
            assert(j>=0 && j<input_vector.size() && "Convolution operator boundary condition violated.");
            current_sum += input_vector[j];
        }
        smoothened_vector.push_back(current_sum/(2.0*smoothing_filter_size - 1.0));
    }
    return smoothened_vector;
}

/// Returns Indices of Start and End of the Lidar Scan for the input truncation angle converage
/// @param scan_msg Lidar scan
/// @param truncation_angle_coverage (in rads)
/// @return Start Index and End Index of Truncated Lidar Scan
std::pair<size_t, size_t> truncated_start_and_end_indices(const sensor_msgs::LaserScan::ConstPtr &scan_msg,
        const double truncation_angle_coverage)
{
    const auto truncated_range_size = static_cast<size_t >(
            (truncation_angle_coverage/(scan_msg->angle_max - scan_msg->angle_min))*scan_msg->ranges.size());
    const size_t start_index = (scan_msg->ranges.size()/2) - (truncated_range_size/2);
    const size_t end_index = (scan_msg->ranges.size()/2) + (truncated_range_size/2);
    return {start_index, end_index};
}

/// Get the least element index in the input_vector
/// @param input_vector
/// @return index of the least element in the input_vector
size_t minimum_element_index(const std::vector<double>& input_vector)
{
    const auto min_value_iterator = std::min_element(input_vector.begin(), input_vector.end());
    return std::distance(input_vector.begin(), min_value_iterator);
}

/// Get the max element index in the input_vector
/// @param input_vector
/// @return index of the max element in the input_vector
size_t maximum_element_index(const std::vector<double>& input_vector)
{
    const auto max_value_iterator = std::max_element(input_vector.begin(), input_vector.end());
    return std::distance(input_vector.begin(), max_value_iterator);
}

/// Finds the maximum gap of non-zero elements in the filtered_ranges vector
/// @param filtered_ranges
/// @return pair of start and end points of the maximum gap
std::pair<size_t, size_t> find_largest_nonzero_sequence(const std::vector<double>& input_vector)
{
    size_t current_start = 0;
    size_t current_size = 0;
    size_t max_start = 0;
    size_t max_size = 0;

    size_t current_index = 0;

    while(current_index < input_vector.size())
    {
        current_start = current_index;
        current_size = 0;
        while(current_index < input_vector.size() && input_vector[current_index] > 0.1)
        {
            current_size++;
            current_index++;
        }
        if(current_size > max_size)
        {
            max_start = current_start;
            max_size = current_size;
            current_size = 0;
        }
        current_index++;
    }
    if(current_size > max_size)
    {
        max_start = current_start;
        max_size = current_size;
    }
    return {max_start, max_start + max_size - 1};
}

/// Zero out all the elements which lie within the bubble radius adjacent to the closest point
/// @param scan_msg - Scan msg type returned by the scan callback
/// @param input_vector
/// @param center
void zero_out_safety_bubble(std::vector<double>* input_vector, const size_t center_index,
                            const double bubble_radius)
{
    const double center_point_distance =  input_vector->at(center_index);
    input_vector->at(center_index) = 0.0;

    size_t current_index = center_index;
    while((current_index < input_vector->size()-1) &&
            (input_vector->at(++current_index) < (center_point_distance + bubble_radius)))
    {
        input_vector->at(current_index) = 0.0;
    }

    current_index = center_index;
    while(current_index > 0 &&
            (input_vector->at(--current_index)) < (center_point_distance + bubble_radius))
    {
        input_vector->at(current_index) = 0.0;
    }
}

}
