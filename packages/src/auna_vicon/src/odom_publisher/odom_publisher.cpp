#include "odom_publisher/odom_publisher.hpp"
#include <rclcpp/qos.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>
#include <cmath>>

OdomPublisherNode::OdomPublisherNode()
: Node("odom_publisher_node")
{
    // Set up QoS profile for real-time, low-latency communication
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(1))
        .reliability(rclcpp::ReliabilityPolicy::BestEffort)
        .durability(rclcpp::DurabilityPolicy::Volatile)
        .deadline(std::chrono::milliseconds(10))
        .lifespan(std::chrono::milliseconds(50));

    // Connect to Vicon Tracker server
    std::string host = "localhost:801";
    
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
      std::bind(&OdomPublisherNode::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "Vicon Odometry Publisher initialized with high-performance settings");
}

void OdomPublisherNode::timer_callback()
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
        auto rotation_output = client_.GetSegmentGlobalRotationQuaternion(subject_name, root_segment_name);

        if (translation_output.Result == ViconDataStreamSDK::CPP::Result::Success &&
            rotation_output.Result == ViconDataStreamSDK::CPP::Result::Success &&
            !translation_output.Occluded && !rotation_output.Occluded)
        {
            if (odom_publishers_.find(subject_name) == odom_publishers_.end()) {
                std::string topic_name = "/vicon/" + subject_name + "/odom";
                odom_publishers_[subject_name] = this->create_publisher<nav_msgs::msg::Odometry>(topic_name, 10);
                RCLCPP_INFO(this->get_logger(), "Created publisher for subject %s on topic %s", subject_name.c_str(), topic_name.c_str());
            }

            nav_msgs::msg::Odometry odom_msg;
            odom_msg.header.stamp = this->now();
            odom_msg.header.frame_id = "vicon_world";
            odom_msg.child_frame_id = subject_name;

            // Fill pose information
            geometry_msgs::msg::Pose current_pose;
            current_pose.position.x = translation_output.Translation[0] / 1000.0;
            current_pose.position.y = translation_output.Translation[1] / 1000.0;
            current_pose.position.z = translation_output.Translation[2] / 1000.0;

            current_pose.orientation.x = rotation_output.Rotation[0];
            current_pose.orientation.y = rotation_output.Rotation[1];
            current_pose.orientation.z = rotation_output.Rotation[2];
            current_pose.orientation.w = rotation_output.Rotation[3];

            odom_msg.pose.pose = current_pose;

            // Calculate and fill twist information
            calculate_twist(subject_name, current_pose, odom_msg.header.stamp, odom_msg);

            // Set covariance matrices (simple diagonal covariance)
            for (int i = 0; i < 36; ++i) {
                odom_msg.pose.covariance[i] = 0.0;
                odom_msg.twist.covariance[i] = 0.0;
            }
            // Position covariance (x, y, z)
            odom_msg.pose.covariance[0] = 0.001;  // x
            odom_msg.pose.covariance[7] = 0.001;  // y
            odom_msg.pose.covariance[14] = 0.001; // z
            // Orientation covariance (roll, pitch, yaw)
            odom_msg.pose.covariance[21] = 0.001; // roll
            odom_msg.pose.covariance[28] = 0.001; // pitch
            odom_msg.pose.covariance[35] = 0.001; // yaw

            // Velocity covariance (vx, vy, vz)
            odom_msg.twist.covariance[0] = 0.01;  // vx
            odom_msg.twist.covariance[7] = 0.01;  // vy
            odom_msg.twist.covariance[14] = 0.01; // vz
            // Angular velocity covariance (wx, wy, wz)
            odom_msg.twist.covariance[21] = 0.01; // wx
            odom_msg.twist.covariance[28] = 0.01; // wy
            odom_msg.twist.covariance[35] = 0.01; // wz
            
            odom_publishers_[subject_name]->publish(odom_msg);
        }
    }
}

void OdomPublisherNode::calculate_twist(const std::string& subject_name, 
                                       const geometry_msgs::msg::Pose& current_pose,
                                       const rclcpp::Time& current_time,
                                       nav_msgs::msg::Odometry& odom_msg)
{
    auto it = previous_poses_.find(subject_name);
    if (it != previous_poses_.end()) {
        // Calculate time difference
        double dt = (current_time - it->second.timestamp).seconds();
        
        if (dt > 0.0 && dt < 1.0) {  // Ensure reasonable time difference
            // Calculate linear velocity
            double dx = current_pose.position.x - it->second.pose.position.x;
            double dy = current_pose.position.y - it->second.pose.position.y;
            double dz = current_pose.position.z - it->second.pose.position.z;
            
            odom_msg.twist.twist.linear.x = dx / dt;
            odom_msg.twist.twist.linear.y = dy / dt;
            odom_msg.twist.twist.linear.z = dz / dt;
            
            // Calculate angular velocity
            tf2::Quaternion q_current, q_previous, q_diff;
            tf2::fromMsg(current_pose.orientation, q_current);
            tf2::fromMsg(it->second.pose.orientation, q_previous);
            
            // Calculate quaternion difference
            q_diff = q_current * q_previous.inverse();
            
            // Convert to angular velocity
            tf2::Vector3 axis = q_diff.getAxis();
            double angle = q_diff.getAngle();
            
            // Handle angle wrapping
            if (angle > M_PI) {
                angle -= 2.0 * M_PI;
            } else if (angle < -M_PI) {
                angle += 2.0 * M_PI;
            }
            
            double angular_speed = angle / dt;
            odom_msg.twist.twist.angular.x = axis.x() * angular_speed;
            odom_msg.twist.twist.angular.y = axis.y() * angular_speed;
            odom_msg.twist.twist.angular.z = axis.z() * angular_speed;
        } else {
            // Invalid time difference, set velocities to zero
            odom_msg.twist.twist.linear.x = 0.0;
            odom_msg.twist.twist.linear.y = 0.0;
            odom_msg.twist.twist.linear.z = 0.0;
            odom_msg.twist.twist.angular.x = 0.0;
            odom_msg.twist.twist.angular.y = 0.0;
            odom_msg.twist.twist.angular.z = 0.0;
        }
    } else {
        // First measurement, set velocities to zero
        odom_msg.twist.twist.linear.x = 0.0;
        odom_msg.twist.twist.linear.y = 0.0;
        odom_msg.twist.twist.linear.z = 0.0;
        odom_msg.twist.twist.angular.x = 0.0;
        odom_msg.twist.twist.angular.y = 0.0;
        odom_msg.twist.twist.angular.z = 0.0;
    }
    
    // Update previous pose for next iteration
    previous_poses_[subject_name].pose = current_pose;
    previous_poses_[subject_name].timestamp = current_time;
}

OdomPublisherNode::~OdomPublisherNode()
{
    if (client_.IsConnected().Connected) {
        client_.Disconnect();
        RCLCPP_INFO(this->get_logger(), "Disconnected from Vicon DataStream");
    }
}
