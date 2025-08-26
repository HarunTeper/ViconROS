#pragma once
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include "/home/ubuntu/vicon/Linux64/DataStreamClient.h"
#include <string>
#include <memory>
#include <unordered_map>

class GpsPublisher : public rclcpp::Node {
public:
  GpsPublisher(const std::string &node_name = "gps_publisher");
  ~GpsPublisher();

private:
  void timer_callback();
  void connect_to_vicon();
  void publish_gps(const std::string &subject_name, const std::string &root_segment_name, const ViconDataStreamSDK::CPP::Output_GetSegmentGlobalTranslation &translation);

  std::unique_ptr<ViconDataStreamSDK::CPP::Client> vicon_client_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool connected_ = false;
  std::unordered_map<std::string, rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr> subject_publishers_;
};
