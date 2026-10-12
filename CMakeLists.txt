cmake_minimum_required(VERSION 3.16.3)

project(sp_vision)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)
message(STATUS "--------------------CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE}--------------------")

# 重装吊射：无 OpenVINO/海康；仍链同济 tools 骨架 + OpenCV(CLI)
option(HEAVY_LOB_ONLY "Only build heavy_lob + offline test" OFF)
if(HEAVY_LOB_ONLY)
  find_package(yaml-cpp REQUIRED)
  find_package(fmt REQUIRED)
  find_package(spdlog REQUIRED)
  find_package(OpenCV REQUIRED)
  include_directories(${PROJECT_SOURCE_DIR})
  include_directories(${OpenCV_INCLUDE_DIRS})
  # 本机无 nlohmann-json3-dev 时用 OpenVINO 自带头文件（只读依赖）
  set(NLOHMANN_JSON_INC /usr/share/openvino/samples/cpp/thirdparty/nlohmann_json/single_include)
  include_directories(${NLOHMANN_JSON_INC})
  # crc 已编进 lob_shot（a5/gimbal_adapter 共用），此处勿再链以免重复符号
  add_library(tools_lob OBJECT
    tools/exiter.cpp
    tools/logger.cpp
    tools/plotter.cpp
  )
  target_include_directories(tools_lob PUBLIC ${PROJECT_SOURCE_DIR} ${NLOHMANN_JSON_INC})
  add_subdirectory(tasks/lob_shot)
  add_executable(heavy_lob src/heavy_lob.cpp $<TARGET_OBJECTS:tools_lob>)
  target_include_directories(heavy_lob PRIVATE ${PROJECT_SOURCE_DIR}/tasks ${NLOHMANN_JSON_INC})
  target_link_libraries(heavy_lob PRIVATE lob_shot yaml-cpp fmt::fmt spdlog::spdlog ${OpenCV_LIBS})
  add_executable(heavy_lob_offline_test tests/heavy_lob_offline_test.cpp)
  target_include_directories(heavy_lob_offline_test PRIVATE ${PROJECT_SOURCE_DIR}/tasks)
  target_link_libraries(heavy_lob_offline_test PRIVATE lob_shot)

  # 钉头离线：不链 OpenVINO/相机，只验 Aimer::nail_mode
  find_package(Eigen3 REQUIRED)
  add_executable(nail_aimer_offline_test
    tests/nail_aimer_offline_test.cpp
    tasks/auto_aim/aimer.cpp
    tasks/auto_aim/target.cpp
    tools/trajectory.cpp
    tools/math_tools.cpp
    tools/extended_kalman_filter.cpp
    tools/logger.cpp
  )
  target_include_directories(nail_aimer_offline_test PRIVATE
    ${PROJECT_SOURCE_DIR} ${EIGEN3_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS})
  target_link_libraries(nail_aimer_offline_test PRIVATE
    yaml-cpp fmt::fmt spdlog::spdlog ${OpenCV_LIBS})

  # 标定离线解算（无相机/无 ROS）；采图仍需全量 build 的 capture
  add_library(tools_calib OBJECT tools/img_tools.cpp tools/math_tools.cpp tools/logger.cpp)
  target_include_directories(tools_calib PUBLIC
    ${PROJECT_SOURCE_DIR} ${EIGEN3_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS} ${NLOHMANN_JSON_INC})
  foreach(tgt calibrate_camera calibrate_handeye calibrate_robotworld_handeye)
    add_executable(${tgt} calibration/${tgt}.cpp $<TARGET_OBJECTS:tools_calib>)
    target_include_directories(${tgt} PRIVATE ${PROJECT_SOURCE_DIR} ${EIGEN3_INCLUDE_DIR})
    target_link_libraries(${tgt} PRIVATE ${OpenCV_LIBS} fmt::fmt yaml-cpp spdlog::spdlog)
  endforeach()
  return()
endif()

find_package(OpenCV REQUIRED)
find_package(fmt REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(spdlog REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(nlohmann_json REQUIRED)
set(OpenVINO_DIR "/opt/intel/openvino_2024.6.0/runtime/cmake/")
find_package(OpenVINO REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR})

# 全量构建时也附带吊射入口（仍不链相机）
add_subdirectory(tasks/lob_shot)
add_executable(heavy_lob src/heavy_lob.cpp)
target_include_directories(heavy_lob PRIVATE ${PROJECT_SOURCE_DIR}/tasks)
add_executable(heavy_lob_offline_test tests/heavy_lob_offline_test.cpp)
target_include_directories(heavy_lob_offline_test PRIVATE ${PROJECT_SOURCE_DIR}/tasks)
target_link_libraries(heavy_lob_offline_test PRIVATE lob_shot)

add_subdirectory(tools)
add_subdirectory(io)
add_subdirectory(tasks/auto_aim)
add_subdirectory(tasks/auto_buff)
add_subdirectory(tasks/omniperception)

# 全量路径：OpenCV(CLI) + tools（Exiter/Plotter/logger）+ spdlog
target_link_libraries(heavy_lob PRIVATE lob_shot yaml-cpp fmt::fmt spdlog::spdlog ${OpenCV_LIBS} tools)

####################src###################
add_executable(standard src/standard.cpp)
add_executable(mt_standard src/mt_standard.cpp)
add_executable(heavy_standard src/heavy_standard.cpp)
add_executable(standard_mpc src/standard_mpc.cpp)
add_executable(auto_aim_debug_mpc src/auto_aim_debug_mpc.cpp)
add_executable(mt_auto_aim_debug src/mt_auto_aim_debug.cpp)
add_executable(auto_buff_debug src/auto_buff_debug.cpp)
add_executable(auto_buff_debug_mpc src/auto_buff_debug_mpc.cpp)
add_executable(uav src/uav.cpp)
add_executable(uav_debug src/uav_debug.cpp)

target_link_libraries(standard ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim auto_buff tools io)
target_link_libraries(mt_standard ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim auto_buff tools io)
target_link_libraries(heavy_standard ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim auto_buff tools io lob_shot)
target_link_libraries(standard_mpc ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim auto_buff tools io)
target_link_libraries(auto_aim_debug_mpc ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim auto_buff tools io)
target_link_libraries(mt_auto_aim_debug ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim auto_buff tools io)
target_link_libraries(auto_buff_debug ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_buff tools io ${CERES_LIBRARIES})
target_link_libraries(auto_buff_debug_mpc ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_buff tools io ${CERES_LIBRARIES})
target_link_libraries(uav ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim auto_buff tools io)
target_link_libraries(uav_debug ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim tools io)

##################calibration################
add_executable(capture calibration/capture.cpp)
add_executable(calibrate_camera calibration/calibrate_camera.cpp)
add_executable(calibrate_handeye calibration/calibrate_handeye.cpp)
add_executable(calibrate_robotworld_handeye calibration/calibrate_robotworld_handeye.cpp)
add_executable(split_video calibration/split_video.cpp)

target_link_libraries(capture ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io lob_shot)
target_link_libraries(calibrate_camera ${OpenCV_LIBS} fmt::fmt yaml-cpp tools)
target_link_libraries(calibrate_handeye ${OpenCV_LIBS} fmt::fmt yaml-cpp tools)
target_link_libraries(calibrate_robotworld_handeye ${OpenCV_LIBS} fmt::fmt yaml-cpp tools)
target_link_libraries(split_video ${OpenCV_LIBS} fmt::fmt tools)

##################tests##################
add_executable(auto_aim_test tests/auto_aim_test.cpp)
add_executable(auto_buff_test tests/auto_buff_test.cpp)
add_executable(camera_detect_test tests/camera_detect_test.cpp)
add_executable(camera_test tests/camera_test.cpp)
add_executable(camera_thread_test tests/camera_thread_test.cpp)
add_executable(cboard_test tests/cboard_test.cpp)
add_executable(fire_test tests/fire_test.cpp)
add_executable(detector_video_test tests/detector_video_test.cpp)
add_executable(gimbal_response_test tests/gimbal_response_test.cpp)
add_executable(multi_usbcamera_test tests/multi_usbcamera_test.cpp)
add_executable(usbcamera_detect_test tests/usbcamera_detect_test.cpp)
add_executable(usbcamera_test tests/usbcamera_test.cpp)
add_executable(handeye_test tests/handeye_test.cpp)
add_executable(dm_test tests/dm_test.cpp)
add_executable(minimum_vision_system tests/minimum_vision_system.cpp)

target_link_libraries(auto_aim_test ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io auto_aim)
target_link_libraries(auto_buff_test ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_buff tools io)
target_link_libraries(camera_detect_test ${OpenCV_LIBS} fmt::fmt yaml-cpp tools auto_aim io)
target_link_libraries(camera_test ${OpenCV_LIBS} fmt::fmt tools io)
target_link_libraries(camera_thread_test ${OpenCV_LIBS} fmt::fmt auto_aim tools io)
target_link_libraries(cboard_test ${OpenCV_LIBS} fmt::fmt tools io)
target_link_libraries(fire_test ${OpenCV_LIBS} fmt::fmt tools io)
target_link_libraries(detector_video_test ${OpenCV_LIBS} fmt::fmt yaml-cpp tools auto_aim)
target_link_libraries(gimbal_response_test ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io)
target_link_libraries(multi_usbcamera_test ${OpenCV_LIBS} fmt::fmt tools io)
target_link_libraries(usbcamera_detect_test ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io auto_aim)
target_link_libraries(usbcamera_test ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io)
target_link_libraries(handeye_test ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io auto_aim)
target_link_libraries(dm_test ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io)
target_link_libraries(minimum_vision_system ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io auto_aim)

add_executable(gimbal_test tests/gimbal_test.cpp)
target_link_libraries(gimbal_test ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim tools io)

add_executable(planner_test tests/planner_test.cpp)
target_link_libraries(planner_test ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim tools io)

add_executable(planner_test_offline tests/planner_test_offline.cpp)
target_link_libraries(planner_test_offline ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim tools io)

# 检查 ROS 环境
find_package(ament_cmake QUIET)
find_package(rclcpp QUIET)
find_package(std_msgs QUIET)
find_package(rosidl_typesupport_cpp QUIET)
find_package(sp_msgs QUIET)
# 只有在找到 ROS 环境时才会添加 sentry 和 publish_test
if(ament_cmake_FOUND AND rclcpp_FOUND AND std_msgs_FOUND AND rosidl_typesupport_cpp_FOUND AND sp_msgs_FOUND)
    # 编译 sentry 和 publish_test
    add_executable(sentry src/sentry.cpp)
    add_executable(sentry_bp src/sentry_bp.cpp)
    add_executable(sentry_debug src/sentry_debug.cpp)
    add_executable(sentry_multithread src/sentry_multithread.cpp)
    add_executable(publish_test tests/publish_test.cpp)
    add_executable(subscribe_test tests/subscribe_test.cpp)
    add_executable(topic_loop_test tests/topic_loop_test.cpp)

    target_link_libraries(sentry ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim omniperception tools io)
    target_link_libraries(sentry_bp ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim omniperception tools io)
    target_link_libraries(sentry_debug ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim omniperception tools io)
    target_link_libraries(sentry_multithread ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim omniperception tools io)
    target_link_libraries(publish_test ${OpenCV_LIBS} fmt::fmt tools io auto_aim)
    target_link_libraries(subscribe_test ${OpenCV_LIBS} fmt::fmt tools io)
    target_link_libraries(topic_loop_test ${OpenCV_LIBS} fmt::fmt tools io)
else()
    message(STATUS "ROS2 environment not found, skipping ROS2-related code.")
endif()
