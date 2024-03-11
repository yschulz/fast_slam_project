#include "fast_slam_ros_core/fast_slam_ros.hpp"

#include <rclcpp/rclcpp.hpp>

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FastSlamRos>());
    rclcpp::shutdown();
    return 0;
}