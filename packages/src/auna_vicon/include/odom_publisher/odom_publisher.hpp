#ifndef ODOM_PUBLISHER_HPP_
#define ODOM_PUBLISHER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include "DataStreamClient.h"
#include <string>
#include <vector>
#include <map>
#include <chrono>

struct PoseHistory {
    geometry_msgs::msg::Pose pose;
    rclcpp::Time timestamp;
};

class OdomPublisherNode : public rclcpp::Node
{
public:
    OdomPublisherNode();
    ~OdomPublisherNode();

private:
    void timer_callback();
    void calculate_twist(const std::string& subject_name, 
                        const geometry_msgs::msg::Pose& current_pose,
                        const rclcpp::Time& current_time,
                        nav_msgs::msg::Odometry& odom_msg);

    ViconDataStreamSDK::CPP::Client client_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::map<std::string, rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr> odom_publishers_;
    std::map<std::string, PoseHistory> previous_poses_;
};

#endif // ODOM_PUBLISHER_HPP_
