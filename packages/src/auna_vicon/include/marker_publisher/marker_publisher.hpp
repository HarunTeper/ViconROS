#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include "/home/ubuntu/vicon/Linux64/DataStreamClient.h"

class ViconMarkerNode : public rclcpp::Node
{
public:
  ViconMarkerNode();

private:
  void timer_callback();
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  ViconDataStreamSDK::CPP::Client client_;
};
