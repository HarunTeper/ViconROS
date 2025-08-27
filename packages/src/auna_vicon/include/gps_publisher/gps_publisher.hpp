#ifndef GPS_PUBLISHER_HPP_
#define GPS_PUBLISHER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include "DataStreamClient.h"
#include <string>
#include <map>

class GpsPublisherNode : public rclcpp::Node
{
public:
    GpsPublisherNode();
    ~GpsPublisherNode();

private:
    void timer_callback();

    ViconDataStreamSDK::CPP::Client client_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::map<std::string, rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr> subject_publishers_;
};

#endif // GPS_PUBLISHER_HPP_

