cmake_minimum_required(VERSION 3.5)
project(auna_vicon)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

set(VICON_SDK_DIR "/home/vscode/workspace/Linux64")

include_directories(
  include
  ${VICON_SDK_DIR}  # Change to your actual Vicon SDK include path
)
link_directories(
  ${VICON_SDK_DIR}
)
add_executable(marker_publisher_main src/marker_publisher_main.cpp src/marker_publisher.cpp)
ament_target_dependencies(marker_publisher_main rclcpp geometry_msgs)

target_link_libraries(marker_publisher_main
  ${rclcpp_LIBRARIES}
  ViconDataStreamSDK_CPP
)

install(TARGETS
  marker_publisher_main
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
