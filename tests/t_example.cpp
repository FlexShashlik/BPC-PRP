#include <iostream>
#include <gtest/gtest.h>
#include <../include/nodes/line_node.hpp>

TEST(LineEstimator, line_estimator_test_1) {
    uint16_t left_value = 0;
    uint16_t right_value = 1024;

    auto result = nodes::LineNode::estimate_descrete_line_pose(left_value, right_value);

    EXPECT_EQ(result, nodes::DiscreteLinePose::LineOnRight);
}

// ...

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
