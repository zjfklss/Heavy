cmake_minimum_required(VERSION 3.8)
project(heavy_lob_shot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(projectile_motion REQUIRED)
find_package(lob_shot_aiming REQUIRED)

add_executable(heavy_lob_manager
  src/manager_node.cpp
  src/main.cpp
)
target_include_directories(heavy_lob_manager PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(heavy_lob_manager PUBLIC cxx_std_17)
ament_target_dependencies(heavy_lob_manager PUBLIC
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  projectile_motion
  lob_shot_aiming
)

add_executable(dry_run_watch src/dry_run_watch.cpp)
target_compile_features(dry_run_watch PUBLIC cxx_std_17)
ament_target_dependencies(dry_run_watch PUBLIC rclcpp std_msgs geometry_msgs)

install(TARGETS heavy_lob_manager dry_run_watch
  DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS
  scripts/gz_gimbal_bridge.py
  scripts/gt_tf.py
  scripts/stl_to_pgm.py
  scripts/field_cloud.py
  scripts/reloc_icp.py
  scripts/encoder_0310.py
  scripts/viewer_0310.py
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config resource rviz DESTINATION share/${PROJECT_NAME})

# 抄 PolarBear rmu_gazebo_simulator/env-hooks：让 model://rmuc_2026 能解析。
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/gazebo.dsv.in")

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_high_gaf test/test_high_gaf.cpp)
  target_include_directories(test_high_gaf PRIVATE include)
  ament_target_dependencies(test_high_gaf projectile_motion)
endif()

ament_package()
