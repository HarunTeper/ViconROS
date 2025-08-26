#include <rclcpp/rclcpp.hpp>
#include "marker_publisher/marker_publisher.hpp"
#include <signal.h>
#include <memory>

// Signal handler for graceful shutdown
std::shared_ptr<ViconMarkerNode> node_ptr;

void signalHandler(int signum) {
    RCLCPP_INFO(rclcpp::get_logger("main"), "Interrupt signal (%d) received. Shutting down gracefully...", signum);
    if (node_ptr) {
        rclcpp::shutdown();
    }
    exit(signum);
}

int main(int argc, char **argv)
{
    // Initialize ROS 2
    rclcpp::init(argc, argv);
    
    // Register signal handler for graceful shutdown
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
    
    try {
        // Create the Vicon marker publisher node
        node_ptr = std::make_shared<ViconMarkerNode>();
        
        // **PERFORMANCE OPTIMIZATION: Use MultiThreadedExecutor for better performance**
        rclcpp::executors::MultiThreadedExecutor executor;
        executor.add_node(node_ptr);
        
        RCLCPP_INFO(node_ptr->get_logger(), "Starting high-performance Vicon marker publisher...");
        
        // Spin the node with optimized executor
        executor.spin();
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "Exception caught: %s", e.what());
        return 1;
    }
    
    // Cleanup
    node_ptr.reset();
    rclcpp::shutdown();
    
    RCLCPP_INFO(rclcpp::get_logger("main"), "Vicon marker publisher shutdown complete");
    return 0;
}