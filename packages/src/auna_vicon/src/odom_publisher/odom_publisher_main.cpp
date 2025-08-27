#include <rclcpp/rclcpp.hpp>
#include "odom_publisher/odom_publisher.hpp"
#include <signal.h>
#include <memory>

std::shared_ptr<OdomPublisherNode> node_ptr;

void signalHandler(int signum) {
    RCLCPP_INFO(rclcpp::get_logger("main"), "Interrupt signal (%d) received. Shutting down gracefully...", signum);
    if (node_ptr) {
        rclcpp::shutdown();
    }
    exit(signum);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
    
    try {
        node_ptr = std::make_shared<OdomPublisherNode>();
        
        rclcpp::executors::MultiThreadedExecutor executor;
        executor.add_node(node_ptr);
        
        RCLCPP_INFO(node_ptr->get_logger(), "Starting high-performance Vicon odometry publisher...");
        
        executor.spin();
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "Exception caught: %s", e.what());
        return 1;
    }
    
    node_ptr.reset();
    rclcpp::shutdown();
    
    RCLCPP_INFO(rclcpp::get_logger("main"), "Vicon odometry publisher shutdown complete");
    return 0;
}
