cmake_minimum_required(VERSION 3.5)
project(auna_vicon)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

set(VICON_SDK_DIR "/home/ubuntu/vicon/Linux64")

include_directories(
  include
  ${VICON_SDK_DIR}  # Change to your actual Vicon SDK include path
)
link_directories(
  ${VICON_SDK_DIR}
)
add_executable(marker_publisher_main src/marker_publisher/marker_publisher_main.cpp src/marker_publisher/marker_publisher.cpp)
add_executable(gps_publisher_main src/gps_publisher/gps_publisher_main.cpp src/gps_publisher/gps_publisher.cpp)
ament_target_dependencies(marker_publisher_main rclcpp geometry_msgs)
ament_target_dependencies(gps_publisher_main rclcpp sensor_msgs)

target_link_libraries(marker_publisher_main
  ${rclcpp_LIBRARIES}
  ViconDataStreamSDK_CPP
)
target_link_libraries(gps_publisher_main
  ${rclcpp_LIBRARIES}
  ViconDataStreamSDK_CPP
)

install(TARGETS
  marker_publisher_main
  gps_publisher_main
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
