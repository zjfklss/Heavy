cmake_minimum_required(VERSION 3.10)
project(armor_detector)

## Use C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

add_definitions(-Wall -Werror)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#######################
## Find dependencies ##
#######################
find_package(ament_cmake_auto REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenVINO REQUIRED COMPONENTS Runtime)
find_package(rcpputils REQUIRED)
find_package(tools REQUIRED)

ament_auto_find_build_dependencies()

# 优先使用动态库
find_library(YAML_CPP_LIBRARY 
  NAMES yaml-cpp libyaml-cpp.so.0.7 libyaml-cpp.so.0.7.0
  PATHS /usr/lib/x86_64-linux-gnu
  NO_DEFAULT_PATH  # 不搜索默认路径，避免找到/usr/local/lib
)

find_path(YAML_CPP_INCLUDE_DIR yaml-cpp/yaml.h
  PATHS /usr/include
  NO_DEFAULT_PATH
)

if(NOT YAML_CPP_LIBRARY)
  # 如果没找到，再尝试搜索其他路径
  find_library(YAML_CPP_LIBRARY NAMES yaml-cpp)
  find_path(YAML_CPP_INCLUDE_DIR yaml-cpp/yaml.h)
endif()

if(NOT YAML_CPP_LIBRARY)
  message(FATAL_ERROR "yaml-cpp library not found!")
endif()

message(STATUS "Using yaml-cpp library: ${YAML_CPP_LIBRARY}")
message(STATUS "Using yaml-cpp include dir: ${YAML_CPP_INCLUDE_DIR}")

###########
## Build ##
###########

ament_auto_add_library(${PROJECT_NAME} SHARED
  DIRECTORY src
)

set_target_properties(${PROJECT_NAME} PROPERTIES
  POSITION_INDEPENDENT_CODE ON
)

target_include_directories(${PROJECT_NAME} PUBLIC 
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
)

target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBS}
  ${rcpputils_LIBRARIES}
  openvino::runtime
  ${YAML_CPP_LIBRARY}
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN rm_auto_aim::ArmorDetectorNode
  EXECUTABLE armor_detector_node
)

#############
## Testing ##
#############

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_copyright
    ament_cmake_uncrustify
    ament_cmake_cpplint
  )
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest)
  ament_add_gtest(test_node_startup test/test_node_startup.cpp)
  target_link_libraries(test_node_startup ${PROJECT_NAME} ${YAML_CPP_LIBRARY})
  
  # ament_add_gtest(test_number_cls test/test_number_cls.cpp)
  # target_link_libraries(test_number_cls ${PROJECT_NAME} ${YAML_CPP_LIBRARY})
endif()

#############
## Install ##
#############

ament_auto_package(
  INSTALL_TO_SHARE
  models
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY data/
  DESTINATION share/${PROJECT_NAME}/data
)

install(DIRECTORY models/
  DESTINATION share/${PROJECT_NAME}/models
)