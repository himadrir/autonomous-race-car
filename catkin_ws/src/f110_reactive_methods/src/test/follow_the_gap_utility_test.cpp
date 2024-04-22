//
// Created by yash on 9/22/19.
//

#include "f110_reactive_methods/utility.h"

#include <gtest/gtest.h>
#include <ros/ros.h>

TEST(find_largest_nonzero_sequence, check_functionality)
{
    const std::vector<double> test_vector_1 = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
    auto start_end_pair = fgm::find_largest_nonzero_sequence(test_vector_1);
    EXPECT_EQ(start_end_pair.first, 0);
    EXPECT_EQ(start_end_pair.second, 9);

    const std::vector<double> test_vector_2 =  {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0.0, 1.0, 1.0, 1.0};
    start_end_pair = fgm::find_largest_nonzero_sequence(test_vector_2);
    EXPECT_EQ(start_end_pair.first, 0);
    EXPECT_EQ(start_end_pair.second, 5);

    const std::vector<double> test_vector_3 =  {1.0, 1.0, 0.0, 2.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 5.0};
    start_end_pair = fgm::find_largest_nonzero_sequence(test_vector_3);
    EXPECT_EQ(start_end_pair.first, 8);
    EXPECT_EQ(start_end_pair.second, 10);

    const std::vector<double> test_vector_4 =  {1.0, 1.0, 1.0, 0.0, 1.0, 0.0, 1.0, 1.0, 1.0, 1.0};
    start_end_pair = fgm::find_largest_nonzero_sequence(test_vector_4);
    EXPECT_EQ(start_end_pair.first, 6);
    EXPECT_EQ(start_end_pair.second, 9);

    const std::vector<double> test_vector_5 =  {1.0, 1.0, 0.0, 2.0, 2.0, 6.0, 2.0, 1.0, 1.0, 1.0, 5.0};
    start_end_pair = fgm::find_largest_nonzero_sequence(test_vector_5);
    EXPECT_EQ(start_end_pair.first, 3);
    EXPECT_EQ(start_end_pair.second, 10);

    const std::vector<double> test_vector_6 = {1.0, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0,
                                               1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
                                               1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
                                               1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
    start_end_pair = fgm::find_largest_nonzero_sequence(test_vector_6);
    EXPECT_EQ(start_end_pair.first, 7);
    EXPECT_EQ(start_end_pair.second, 39);

    const std::vector<double> test_vector_7 = {5.0, 0.5, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
                                               1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
                                               1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0.0, 0.0, 1.0, 1.0,
                                               1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
    start_end_pair = fgm::find_largest_nonzero_sequence(test_vector_7);
    EXPECT_EQ(start_end_pair.first, 0);
    EXPECT_EQ(start_end_pair.second, 25);
}

TEST(minimum_element_index, check_functionality)
{
    const std::vector<double> test_vector_1 = {1.0, 2.0, 1.0, 3.0, 0.5, 1.0, 1.0, 1.0, 1.0, 1.0};
    auto min_index = fgm::minimum_element_index(test_vector_1);
    EXPECT_EQ(min_index, 4);

    const std::vector<double> test_vector_2 = {22.0, 1.0, 7.0, 1.0, 2.0, 6.0, 5.0, 5.0, 1.0, 1.0};
    min_index = fgm::minimum_element_index(test_vector_2);
    EXPECT_EQ(min_index, 1);

    const std::vector<double> test_vector_3 = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0.7, 1.0, 1.0, 0.5};
    min_index = fgm::minimum_element_index(test_vector_3);
    EXPECT_EQ(min_index, 9);

    const std::vector<double> test_vector_4 = {0.0, 1.0, 0.2, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
    min_index = fgm::minimum_element_index(test_vector_4);
    EXPECT_EQ(min_index, 0);

    const std::vector<double> test_vector_5 = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 11.0};
    min_index = fgm::minimum_element_index(test_vector_5);
    EXPECT_EQ(min_index, 0);
}

TEST(maximum_element_index, check_functionality)
{
    const std::vector<double> test_vector_1 = {1.0, 2.0, 1.0, 3.0, 0.5, 1.0, 1.0, 1.0, 1.0, 1.0};
    auto min_index = fgm::maximum_element_index(test_vector_1);
    EXPECT_EQ(min_index, 3);

    const std::vector<double> test_vector_2 = {22.0, 1.0, 7.0, 1.0, 2.0, 6.0, 5.0, 5.0, 1.0, 1.0};
    min_index = fgm::maximum_element_index(test_vector_2);
    EXPECT_EQ(min_index, 0);

    const std::vector<double> test_vector_3 = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0.7, 1.0, 1.0, 0.5};
    min_index = fgm::maximum_element_index(test_vector_3);
    EXPECT_EQ(min_index, 0);

    const std::vector<double> test_vector_4 = {0.0, 1.0, 0.2, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
    min_index = fgm::maximum_element_index(test_vector_4);
    EXPECT_EQ(min_index, 1);

    const std::vector<double> test_vector_5 = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 11.0};
    min_index = fgm::maximum_element_index(test_vector_5);
    EXPECT_EQ(min_index, 9);
}

TEST(zero_out_bubble, check_functionality)
{
    std::vector<double> test_vector = {1.0, 2.0, 1.0, 3.0, 0.5, 1.0, 1.0, 1.0, 1.0, 1.0};
    std::vector<double> output_vector = {1.0, 2.0, 1.0, 3.0, 0.0, 1.0, 1.0, 1.0, 1.0, 1.0};
    fgm::zero_out_safety_bubble(&test_vector, 4, 0.4);
    EXPECT_EQ(test_vector, output_vector);

    test_vector = {1.0, 2.0, 1.0, 3.0, 0.5, 1.0, 0.9, 1.0, 1.0, 1.0};
    output_vector = {1.0, 2.0, 1.0, 3.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    fgm::zero_out_safety_bubble(&test_vector, 4, 1.0);
    EXPECT_EQ(test_vector, output_vector);

    test_vector = {1.0, 2.0, 1.0, 3.0, 0.5, 1.0, 1.0, 1.0, 1.0, 1.0};
    output_vector = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    fgm::zero_out_safety_bubble(&test_vector, 4, 2.9);
    EXPECT_EQ(test_vector, output_vector);

    test_vector = {22.0, 1.0, 7.0, 1.0, 2.0, 6.0, 5.0, 5.0, 1.0, 1.0};
    output_vector = {22.0, 0.0, 7.0, 1.0, 2.0, 6.0, 5.0, 5.0, 1.0, 1.0};
    fgm::zero_out_safety_bubble(&test_vector, 1, 0.7);
    EXPECT_EQ(test_vector, output_vector);

    test_vector = {1.0, 0.5, 0.1, 0.1, 0.1, 0.1, 0.5, 1.0, 1.0, 1.0,
                   1.0, 1.0, 1.0, 1.2, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
                   1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
                   1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
    output_vector = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                     0.0, 0.0, 0.0, 1.2, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
                     1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
                     1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
    fgm::zero_out_safety_bubble(&test_vector, 2, 1.0);
    EXPECT_EQ(test_vector, output_vector);

    test_vector = {5.0, 0.5, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
                   1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
                   0.1, 0.2, 0.1, 0.2, 0.1, 0.2, 0.1, 0.2, 0.3, 0.5,
                   0.1, 0.2, 0.1, 0.2, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
    output_vector = {5.0, 0.5, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
                     1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
                     0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                     0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
    fgm::zero_out_safety_bubble(&test_vector, 20, 0.5);
    EXPECT_EQ(test_vector, output_vector);
}

TEST(get_steering_angle_from_range_index, check_functionality)
{

}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
