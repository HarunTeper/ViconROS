#include "gps_publisher/gps_publisher.hpp"
#include <rclcpp/context.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <chrono>
#include <vector>

using namespace std::chrono_literals;

GpsPublisher::GpsPublisher(const std::string &node_name)
    : Node(node_name) {
  vicon_client_ = std::make_unique<ViconDataStreamSDK::CPP::Client>();
  timer_ = this->create_wall_timer(1000ms, std::bind(&GpsPublisher::timer_callback, this));
  connect_to_vicon();
  RCLCPP_INFO(this->get_logger(), "GPS Publisher Node Initialized");
}

GpsPublisher::~GpsPublisher() {
  if (vicon_client_ && connected_) {
    vicon_client_->Disconnect();
  }
}

void GpsPublisher::connect_to_vicon() {
  RCLCPP_INFO(this->get_logger(), "Attempting to connect to the Vicon system...");
  if (vicon_client_->IsConnected().Connected) return;
  std::string host = "localhost:801";
  auto result = vicon_client_->Connect(host);
  connected_ = result.Result == ViconDataStreamSDK::CPP::Result::Success;
  if (!connected_) {
    RCLCPP_ERROR(this->get_logger(), "Failed to connect to Vicon system");
    rclcpp::shutdown();
    RCLCPP_ERROR(this->get_logger(), "Shutting down due to connection failure");
    return;
  } else {
    vicon_client_->EnableSegmentData();
    vicon_client_->SetStreamMode(ViconDataStreamSDK::CPP::StreamMode::ClientPull);
    RCLCPP_INFO(this->get_logger(), "Connected to the Vicon system");
  }
}

void GpsPublisher::timer_callback() {
  if (!connected_) {
    RCLCPP_INFO(this->get_logger(), "Not connected to Vicon, attempting to reconnect...");
    connect_to_vicon();
    RCLCPP_INFO(this->get_logger(), "Reconnection attempt complete");
    return;
  }
  vicon_client_->GetFrame();
  auto OutputGSC= vicon_client_->GetSubjectCount();
  RCLCPP_INFO(this->get_logger(), "Number of subjects: %u", OutputGSC.SubjectCount);
  auto subject_count = OutputGSC.SubjectCount;
  for (unsigned int i = 0; i < subject_count; ++i) {
    std::string subject_name = vicon_client_->GetSubjectName(i).SubjectName;
    RCLCPP_INFO(this->get_logger(), "Subject %u: %s", i, subject_name.c_str());
    auto root_segment_output = vicon_client_->GetSubjectRootSegmentName(subject_name);
    if (root_segment_output.Result == ViconDataStreamSDK::CPP::Result::Success) {
      std::string root_segment_name = root_segment_output.SegmentName;
      RCLCPP_INFO(this->get_logger(), "Root segment for subject %s: %s", subject_name.c_str(), root_segment_name.c_str());
      auto translation_output = vicon_client_->GetSegmentGlobalTranslation(root_segment_name, root_segment_name);
      if (translation_output.Result != ViconDataStreamSDK::CPP::Result::Success) {
        RCLCPP_WARN(this->get_logger(), "Failed to get global translation for segment %s", root_segment_name.c_str());
        continue;
      }
      // Create publisher for this subject if it doesn't exist
      if (subject_publishers_.find(subject_name) == subject_publishers_.end()) {
        std::string topic_name = "/"+subject_name+"/gps/fix";
        subject_publishers_[subject_name] = this->create_publisher<sensor_msgs::msg::NavSatFix>(topic_name, 10);
        RCLCPP_INFO(this->get_logger(), "Created publisher for subject %s on topic %s", subject_name.c_str(), topic_name.c_str());
      }
      publish_gps(subject_name, root_segment_name, translation_output);
    } else {
      RCLCPP_WARN(this->get_logger(), "Failed to get root segment for subject %s", subject_name.c_str());
    }
  }
}

void GpsPublisher::publish_gps(const std::string &subject_name, const std::string &root_segment_name, const ViconDataStreamSDK::CPP::Output_GetSegmentGlobalTranslation &translation) {
  // Directly use the translation as x, y, z
  double lat = translation.Translation[0] / 1000.0; // Convert mm to meters
  double lon = translation.Translation[1] / 1000.0; // Convert mm to meters
  double alt = translation.Translation[2] / 1000.0; // Convert mm to meters
  RCLCPP_INFO(this->get_logger(), "Publishing GPS data for segment %s: lat=%.6f, lon=%.6f, alt=%.6f", 
              root_segment_name.c_str(), lat, lon, alt);

  // Set the covariance at 1 5 9 to 0.001
  std::vector<double> covariance(9, 0.0);
  covariance[0] = 0.001; // Covariance for latitude
  covariance[4] = 0.001; // Covariance for longitude
  covariance[8] = 0.001; // Covariance for altitude
  // Create and publish the NavSatFix message

  sensor_msgs::msg::NavSatFix msg;
  msg.header.stamp = this->now();
  msg.header.frame_id = "gps_link";
  msg.latitude = lat;
  msg.longitude = lon;
  msg.altitude = alt;
  std::copy(covariance.begin(), covariance.end(), msg.position_covariance.begin());
  msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
  msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
  msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;

  // Publish to the subject-specific publisher
  auto it = subject_publishers_.find(subject_name);
  if (it != subject_publishers_.end()) {
    it->second->publish(msg);
  } else {
    RCLCPP_WARN(this->get_logger(), "No publisher found for subject %s", subject_name.c_str());
  }
}
