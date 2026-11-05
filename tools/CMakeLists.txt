add_library(tools STATIC
  extended_kalman_filter.cpp
  img_tools.cpp
  math_tools.cpp
  logger.cpp
  plotter.cpp
  trajectory.cpp
)
target_include_directories(tools PUBLIC ${PROJECT_SOURCE_DIR} ${OpenCV_INCLUDE_DIRS})
target_link_libraries(tools PUBLIC
  Eigen3::Eigen ${OpenCV_LIBS} yaml-cpp fmt::fmt spdlog::spdlog
  nlohmann_json::nlohmann_json Threads::Threads
)
