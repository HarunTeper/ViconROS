cmake_minimum_required(VERSION 3.5)
project(auna_vicon)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

set(VICON_SDK_DIR "/home/vscode/workspace/Linux64")

include_directories(
  include
  ${VICON_SDK_DIR}
)
link_directories(
  ${VICON_SDK_DIR}
)
add_executable(vicon_ros src/vicon_ros.cpp)
ament_target_dependencies(vicon_ros rclcpp geometry_msgs)

target_link_libraries(vicon_ros
  ${rclcpp_LIBRARIES}
  ViconDataStreamSDK_CPP
)

install(TARGETS
  vicon_ros
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
