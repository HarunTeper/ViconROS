#include "marker_publisher/marker_publisher.hpp"

ViconMarkerNode::ViconMarkerNode()
: Node("vicon_marker_node")
{
  // Create a ROS2 publisher to publish marker positions as PoseArray
  marker_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("vicon_markers", 10);

  // Connect to the Vicon Tracker server
  std::string host = "localhost:801"; // Replace with your actual Vicon IP address
  if (client_.Connect(host).Result != ViconDataStreamSDK::CPP::Result::Success) {
    RCLCPP_ERROR(this->get_logger(), "Failed to connect to Vicon DataStream: %s", host.c_str());
    rclcpp::shutdown();
    return;
  }

  // Enable marker data stream
  client_.EnableMarkerData();
  
  // Set stream mode to ClientPull (pulls frame manually)
  client_.SetStreamMode(ViconDataStreamSDK::CPP::StreamMode::ClientPull);

  // Set axis mapping (Forward-Left-Up is standard ROS convention)
  client_.SetAxisMapping(
    ViconDataStreamSDK::CPP::Direction::Forward,
    ViconDataStreamSDK::CPP::Direction::Left,
    ViconDataStreamSDK::CPP::Direction::Up);

  RCLCPP_INFO(this->get_logger(), "Successfully connected to Vicon DataStream.");

  // Create a timer to call the callback every 10 milliseconds
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(10),
    std::bind(&ViconMarkerNode::timer_callback, this));
}

void ViconMarkerNode::timer_callback()
{
  // Pull the latest frame from Vicon
  if (client_.GetFrame().Result != ViconDataStreamSDK::CPP::Result::Success) {
    RCLCPP_WARN(this->get_logger(), "Failed to get Vicon frame.");
    return;
  }

  geometry_msgs::msg::PoseArray pose_array;
  pose_array.header.stamp = this->now();               // Timestamp
  pose_array.header.frame_id = "vicon_world";          // Coordinate frame

  unsigned int subject_count = client_.GetSubjectCount().SubjectCount;
  for (unsigned int i = 0; i < subject_count; ++i) {
    std::string subject_name = client_.GetSubjectName(i).SubjectName;
    unsigned int marker_count = client_.GetMarkerCount(subject_name).MarkerCount;

    for (unsigned int j = 0; j < marker_count; ++j) {
      std::string marker_name = client_.GetMarkerName(subject_name, j).MarkerName;
      auto result = client_.GetMarkerGlobalTranslation(subject_name, marker_name);

      if (result.Result == ViconDataStreamSDK::CPP::Result::Success) {
        geometry_msgs::msg::Pose pose;
        pose.position.x = result.Translation[0] / 1000.0;  // Convert mm to meters
        pose.position.y = result.Translation[1] / 1000.0;
        pose.position.z = result.Translation[2] / 1000.0;
        pose.orientation.w = 1.0;                          // No rotation

        pose_array.poses.push_back(pose);

        // Log marker position
        RCLCPP_INFO(this->get_logger(), "Marker: %s Position: [%.3f, %.3f, %.3f]",
          marker_name.c_str(), pose.position.x, pose.position.y, pose.position.z);
      }
    }
  }

  // Publish the array of marker poses
  marker_pub_->publish(pose_array);
  RCLCPP_INFO(this->get_logger(), "Publishing %zu markers", pose_array.poses.size());
}
