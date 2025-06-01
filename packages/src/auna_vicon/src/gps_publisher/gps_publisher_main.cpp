#include "gps_publisher/gps_publisher.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GpsPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
