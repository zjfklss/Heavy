cmake_minimum_required(VERSION 3.8)
project(doorlock_sniper)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(serial_test REQUIRED)

get_target_property(CV_BRIDGE_INCLUDE_DIRS cv_bridge::cv_bridge INTERFACE_INCLUDE_DIRECTORIES)
include_directories(${CV_BRIDGE_INCLUDE_DIRS})
rosidl_generate_interfaces(${PROJECT_NAME}
  msg/VideoPacket.msg
  DEPENDENCIES std_msgs
)
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${serial_test_INCLUDE_DIRS}
)

add_library(video_encoder_node SHARED
  src/video_encoder_node.cpp
)

target_compile_features(video_encoder_node PUBLIC cxx_std_17)

ament_target_dependencies(video_encoder_node
  rclcpp
  rclcpp_components
  sensor_msgs
  cv_bridge
  std_msgs
  OpenCV
  serial_test
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(video_encoder_node ${cpp_typesupport_target})

rclcpp_components_register_nodes(video_encoder_node "doorlock_sniper::VideoEncoderNode")

install(TARGETS video_encoder_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY msg/
  DESTINATION share/${PROJECT_NAME}/msg
)

ament_export_dependencies(
  rclcpp
  rclcpp_components
  sensor_msgs
  cv_bridge
  OpenCV
  serial_test
)

ament_package()
