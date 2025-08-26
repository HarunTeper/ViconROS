#ifndef MARKER_PUBLISHER_HPP_
#define MARKER_PUBLISHER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include "DataStreamClient.h"  // Vicon DataStream SDK
#include <memory>
#include <string>

class ViconMarkerNode : public rclcpp::Node
{
public:
    ViconMarkerNode();
    ~ViconMarkerNode();

private:
    void timer_callback();
    
    // ROS 2 components
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr marker_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    // Vicon DataStream client
    ViconDataStreamSDK::CPP::Client client_;
    
    // **OPTIMIZATION: Pre-allocated pose array to avoid memory reallocations**
    geometry_msgs::msg::PoseArray pose_array_;
};

#endif // MARKER_PUBLISHER_HPP_