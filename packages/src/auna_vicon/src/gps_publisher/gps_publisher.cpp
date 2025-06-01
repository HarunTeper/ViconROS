#include "gps_publisher/gps_publisher.hpp"
#include <chrono>
#include <vector>

using namespace std::chrono_literals;

GpsPublisher::GpsPublisher(const std::string &node_name)
    : Node(node_name) {
  gps_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("/gps/fix", 10);
  vicon_client_ = std::make_unique<ViconDataStreamSDK::CPP::Client>();
  timer_ = this->create_wall_timer(100ms, std::bind(&GpsPublisher::timer_callback, this));
  connect_to_vicon();
}

GpsPublisher::~GpsPublisher() {
//   if (vicon_client_ && connected_) {
//     vicon_client_->Disconnect();
//   }
}

void GpsPublisher::connect_to_vicon() {
//   if (vicon_client_->IsConnected().Connected) return;
//   auto result = vicon_client_->Connect("localhost:801");
//   connected_ = result.Result == ViconDataStreamSDK::CPP::Result::Success;
//   if (!connected_) {
//     RCLCPP_ERROR(this->get_logger(), "Failed to connect to Vicon system");
//   } else {
//     vicon_client_->EnableSegmentData();
//     vicon_client_->EnableSubjectFilter();
//     vicon_client_->SetStreamMode(ViconDataStreamSDK::CPP::StreamMode::ClientPull);
//     RCLCPP_INFO(this->get_logger(), "Connected to Vicon system");
//   }
}

void GpsPublisher::timer_callback() {
//   if (!connected_) {
//     connect_to_vicon();
//     return;
//   }
//   vicon_client_->GetFrame();
//   auto subject_count = vicon_client_->GetSubjectCount().SubjectCount;
//   for (unsigned int i = 0; i < subject_count; ++i) {
//     std::string subject_name = vicon_client_->GetSubjectName(i).SubjectName;
//     auto translation = vicon_client_->GetGlobalTranslation(subject_name);
//     if (translation.Result == ViconDataStreamSDK::CPP::Result::Success) {
//       publish_gps(subject_name, translation);
//     }
//   }
}

// void GpsPublisher::publish_gps(const std::string &subject_name, const ViconDataStreamSDK::CPP::Output_GetGlobalTranslation &translation) {
//   sensor_msgs::msg::NavSatFix msg;
//   msg.header.stamp = this->now();
//   msg.header.frame_id = subject_name;
//   // Dummy conversion: Vicon XYZ to GPS (lat/lon/alt). Replace with real conversion as needed.
//   msg.latitude = translation.Translation[0] * 1e-5;
//   msg.longitude = translation.Translation[1] * 1e-5;
//   msg.altitude = translation.Translation[2] * 1e-3;
//   msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
//   msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
//   gps_pub_->publish(msg);
// }
