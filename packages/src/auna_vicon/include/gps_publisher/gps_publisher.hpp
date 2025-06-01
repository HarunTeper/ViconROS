#pragma once
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include "/home/vscode/workspace/Linux64/DataStreamClient.h"
#include <string>
#include <memory>

class GpsPublisher : public rclcpp::Node {
public:
  GpsPublisher(const std::string &node_name = "gps_publisher");
  ~GpsPublisher();

private:
  void timer_callback();
  void connect_to_vicon();
//   void publish_gps(const std::string &subject_name, const ViconDataStreamSDK::CPP::Output_GetGlobalTranslation &translation);

  std::unique_ptr<ViconDataStreamSDK::CPP::Client> vicon_client_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool connected_ = false;
};
