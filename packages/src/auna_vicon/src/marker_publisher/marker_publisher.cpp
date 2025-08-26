#include "marker_publisher/marker_publisher.hpp"
#include <rclcpp/qos.hpp>

ViconMarkerNode::ViconMarkerNode()
: Node("vicon_marker_node")
{
    // Set up QoS profile for real-time, low-latency communication
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(1))  // Keep only latest message
        .reliability(rclcpp::ReliabilityPolicy::BestEffort)  // Fastest delivery
        .durability(rclcpp::DurabilityPolicy::Volatile)      // Don't persist messages
        .deadline(std::chrono::milliseconds(10))             // Max 10ms between messages
        .lifespan(std::chrono::milliseconds(50));            // Messages valid for 50ms

    // Create publisher with optimized QoS
    marker_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
        "vicon_markers", qos_profile);

    // Connect to Vicon Tracker server
    std::string host = "localhost:801"; // Replace with your actual Vicon IP address
    
    RCLCPP_INFO(this->get_logger(), "Connecting to Vicon DataStream: %s", host.c_str());
    
    if (client_.Connect(host).Result != ViconDataStreamSDK::CPP::Result::Success) {
        RCLCPP_ERROR(this->get_logger(), "Failed to connect to Vicon DataStream: %s", host.c_str());
        rclcpp::shutdown();
        return;
    }

    // Set ServerPush mode for lowest latency
    auto stream_result = client_.SetStreamMode(ViconDataStreamSDK::CPP::StreamMode::ServerPush);
    if (stream_result.Result == ViconDataStreamSDK::CPP::Result::Success) {
        RCLCPP_INFO(this->get_logger(), "Set stream mode to ServerPush for lowest latency");
    } else {
        RCLCPP_WARN(this->get_logger(), "Failed to set ServerPush mode, using default");
    }

    // Set minimal buffer size for lowest latency
    client_.SetBufferSize(1);  // Only buffer 1 frame for minimal delay
    RCLCPP_INFO(this->get_logger(), "Set buffer size to 1 for minimal latency");

    // Enable marker data stream with high performance
    client_.EnableMarkerData();

    // Set axis mapping to ROS standard (Forward-Left-Up)
    auto axis_result = client_.SetAxisMapping(
        ViconDataStreamSDK::CPP::Direction::Forward,
        ViconDataStreamSDK::CPP::Direction::Left,
        ViconDataStreamSDK::CPP::Direction::Up);
    
    if (axis_result.Result == ViconDataStreamSDK::CPP::Result::Success) {
        RCLCPP_INFO(this->get_logger(), "Set axis mapping to ROS standard (FLU)");
    }

    RCLCPP_INFO(this->get_logger(), "Successfully connected to Vicon DataStream");

    // Use high-frequency fallback timer for maximum responsiveness
    RCLCPP_INFO(this->get_logger(), "Setting timer to 3030 microseconds (330Hz)");
    timer_ = this->create_wall_timer(
      std::chrono::microseconds(3030),  // ~3.03ms = 330Hz
      std::bind(&ViconMarkerNode::timer_callback, this));

    // Pre-allocate pose array to avoid reallocations
    pose_array_.poses.reserve(30);  // Reserve space for up to 30 markers

    RCLCPP_INFO(this->get_logger(), "Vicon Marker Publisher initialized with high-performance settings");
}

void ViconMarkerNode::timer_callback()
{
    // Use GetFrame() for ServerPush mode - this is non-blocking in ServerPush
    auto frame_result = client_.GetFrame();
    if (frame_result.Result != ViconDataStreamSDK::CPP::Result::Success) {
        // In ServerPush mode, this just means no new frame is available yet
        return;  // Don't log warnings as this is normal in high-frequency polling
    }

    // Reuse the same pose array to avoid memory allocation
    pose_array_.poses.clear();  // Clear but keep capacity
    pose_array_.header.stamp = this->now();
    pose_array_.header.frame_id = "vicon_world";

    // Get all subjects and their markers
    unsigned int subject_count = client_.GetSubjectCount().SubjectCount;
    for (unsigned int i = 0; i < subject_count; ++i) {
        std::string subject_name = client_.GetSubjectName(i).SubjectName;
        unsigned int marker_count = client_.GetMarkerCount(subject_name).MarkerCount;
        
        for (unsigned int j = 0; j < marker_count; ++j) {
            std::string marker_name = client_.GetMarkerName(subject_name, j).MarkerName;
            auto result = client_.GetMarkerGlobalTranslation(subject_name, marker_name);
            
            if (result.Result == ViconDataStreamSDK::CPP::Result::Success && !result.Occluded) {
                geometry_msgs::msg::Pose pose;

                // Direct assignment without intermediate variables
                pose.position.x = result.Translation[0] * 0.001; // mm to m conversion
                pose.position.y = result.Translation[1] * 0.001;
                pose.position.z = result.Translation[2] * 0.001;
                
                // Set default orientation (no rotation)
                pose.orientation.x = 0.0;
                pose.orientation.y = 0.0;
                pose.orientation.z = 0.0;
                pose.orientation.w = 1.0;
                
                pose_array_.poses.push_back(std::move(pose));
            }
        }
    }

    // Only publish if we have marker data
    if (!pose_array_.poses.empty()) {
        marker_pub_->publish(pose_array_);
    }
}

ViconMarkerNode::~ViconMarkerNode()
{
    if (client_.IsConnected().Connected) {
        client_.Disconnect();
        RCLCPP_INFO(this->get_logger(), "Disconnected from Vicon DataStream");
    }
}