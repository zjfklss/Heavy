cmake_minimum_required(VERSION 3.8)
project(serial_test)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(serial_driver REQUIRED)
find_package(io_context REQUIRED)

include_directories(include)

# 创建共享库
add_library(serial_test SHARED
  src/serial_test.cpp
  src/crc.cpp
)

target_compile_features(serial_test PUBLIC cxx_std_17)

ament_target_dependencies(serial_test
  rclcpp
  rclcpp_components
  std_msgs
  serial_driver
  io_context
)

rclcpp_components_register_nodes(serial_test "serial_test::SerialTest")

# 安装库
install(TARGETS serial_test
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# 安装头文件
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

# 创建可执行文件
add_executable(serial_test_node src/main.cpp)
target_link_libraries(serial_test_node serial_test)
ament_target_dependencies(serial_test_node rclcpp)

install(TARGETS serial_test_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(
  rclcpp
  rclcpp_components
  std_msgs
  serial_driver
  io_context
)

ament_export_include_directories(include)
ament_export_libraries(serial_test)

ament_package()