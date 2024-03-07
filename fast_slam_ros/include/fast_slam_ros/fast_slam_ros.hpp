#ifndef FAST_SLAM_ROS_HPP
#define FAST_SLAM_ROS_HPP

#include <rclcpp/rclcpp.hpp>
#include <fast_slam/ParticleSet.hpp>
#include <fast_slam/MeasurementSet.hpp>
#include <fast_slam/Helper.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/time.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

using namespace std::chrono_literals;

class FastSlamRos : public rclcpp::Node{
public:
    FastSlamRos();
    ~FastSlamRos();

private:
    void timerCallback();

    void broadCastTransform();
    void publishParticles();
    void publishMap();
    void publishPath();
    void publishDistribution();

    void landmarkCallback(const std::shared_ptr<geometry_msgs::msg::PoseArray> msg);
    void odomCallback(const std::shared_ptr<nav_msgs::msg::Odometry> msg);

    double getYaw(tf2::Quaternion q);

    fastslam::ParticleSet p_set_;
    // fastslam::MeasurementSet m_set_;

    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr landmark_maesurement_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr particle_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr landmark_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr distribution_pub_;

    rclcpp::TimerBase::SharedPtr timer_;

    double d_min_angular_;
    double d_min_linear_;

    std::string map_frame_;
    std::string odom_frame_;
    std::string robot_frame_;

    geometry_msgs::msg::PoseArray latest_landmarks_;
    geometry_msgs::msg::TransformStamped last_transform_;

    geometry_msgs::msg::Pose last_pose_;
    geometry_msgs::msg::Pose current_pose_;

    nav_msgs::msg::Path path_;
    rclcpp::Time time_old_;

    bool initialized_ = false;

};

#endif