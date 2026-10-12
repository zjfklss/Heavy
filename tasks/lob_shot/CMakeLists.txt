cmake_minimum_required(VERSION 3.16)

add_library(lob_shot STATIC
  manager.cpp
  lob_serial.cpp
  a5_packet.cpp
  ../../tools/crc.cpp
  projectile_motion/gaf_projectile_solver.cpp
  projectile_motion/iterative_projectile_tool.cpp
  projectile_motion/gravity_projectile_solver.cpp
)
target_include_directories(lob_shot PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/..
  ${CMAKE_CURRENT_SOURCE_DIR}/../..
)
