#include "gps_publisher/gps_publisher.hpp"
#include <rclcpp/qos.hpp>
#include <vector>

GpsPublisherNode::GpsPublisherNode()
    : Node("gps_publisher_node")
{
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(1))
        .reliability(rclcpp::ReliabilityPolicy::BestEffort)
        .durability(rclcpp::DurabilityPolicy::Volatile)
        .deadline(std::chrono::milliseconds(10))
        .lifespan(std::chrono::milliseconds(50));

    std::string host = "localhost:801";
    
    RCLCPP_INFO(this->get_logger(), "Connecting to Vicon DataStream: %s", host.c_str());
    
    if (client_.Connect(host).Result != ViconDataStreamSDK::CPP::Result::Success) {
        RCLCPP_ERROR(this->get_logger(), "Failed to connect to Vicon DataStream: %s", host.c_str());
        rclcpp::shutdown();
        return;
    }

    auto stream_result = client_.SetStreamMode(ViconDataStreamSDK::CPP::StreamMode::ServerPush);
    if (stream_result.Result == ViconDataStreamSDK::CPP::Result::Success) {
        RCLCPP_INFO(this->get_logger(), "Set stream mode to ServerPush for lowest latency");
    } else {
        RCLCPP_WARN(this->get_logger(), "Failed to set ServerPush mode, using default");
    }

    client_.SetBufferSize(1);
    RCLCPP_INFO(this->get_logger(), "Set buffer size to 1 for minimal latency");

    client_.EnableSegmentData();

    auto axis_result = client_.SetAxisMapping(
        ViconDataStreamSDK::CPP::Direction::Forward,
        ViconDataStreamSDK::CPP::Direction::Left,
        ViconDataStreamSDK::CPP::Direction::Up);
    
    if (axis_result.Result == ViconDataStreamSDK::CPP::Result::Success) {
        RCLCPP_INFO(this->get_logger(), "Set axis mapping to ROS standard (FLU)");
    }

    RCLCPP_INFO(this->get_logger(), "Successfully connected to Vicon DataStream");

    RCLCPP_INFO(this->get_logger(), "Setting timer to 3030 microseconds (330Hz)");
    timer_ = this->create_wall_timer(
      std::chrono::microseconds(3030),
      std::bind(&GpsPublisherNode::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "Vicon GPS Publisher initialized with high-performance settings");
}

GpsPublisherNode::~GpsPublisherNode()
{
    if (client_.IsConnected().Connected) {
        client_.Disconnect();
        RCLCPP_INFO(this->get_logger(), "Disconnected from Vicon DataStream");
    }
}

void GpsPublisherNode::timer_callback()
{
    auto frame_result = client_.GetFrame();
    if (frame_result.Result != ViconDataStreamSDK::CPP::Result::Success) {
        return;
    }

    unsigned int subject_count = client_.GetSubjectCount().SubjectCount;
    for (unsigned int i = 0; i < subject_count; ++i) {
        std::string subject_name = client_.GetSubjectName(i).SubjectName;
        auto root_segment_output = client_.GetSubjectRootSegmentName(subject_name);
        if (root_segment_output.Result != ViconDataStreamSDK::CPP::Result::Success) {
            continue;
        }
        std::string root_segment_name = root_segment_output.SegmentName;

        auto translation_output = client_.GetSegmentGlobalTranslation(subject_name, root_segment_name);
        if (translation_output.Result == ViconDataStreamSDK::CPP::Result::Success && !translation_output.Occluded)
        {
            if (subject_publishers_.find(subject_name) == subject_publishers_.end()) {
                std::string topic_name = "/vicon/" + subject_name + "/gps/fix";
                subject_publishers_[subject_name] = this->create_publisher<sensor_msgs::msg::NavSatFix>(topic_name, 10);
                RCLCPP_INFO(this->get_logger(), "Created publisher for subject %s on topic %s", subject_name.c_str(), topic_name.c_str());
            }

            sensor_msgs::msg::NavSatFix msg;
            msg.header.stamp = this->now();
            msg.header.frame_id = "gps_link";
            msg.latitude = translation_output.Translation[0] / 1000.0;
            msg.longitude = translation_output.Translation[1] / 1000.0;
            msg.altitude = translation_output.Translation[2] / 1000.0;
            
            std::vector<double> covariance(9, 0.0);
            covariance[0] = 0.001;
            covariance[4] = 0.001;
            covariance[8] = 0.001;
            std::copy(covariance.begin(), covariance.end(), msg.position_covariance.begin());
            msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
            msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
            msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;

            subject_publishers_[subject_name]->publish(msg);
        }
    }
}

