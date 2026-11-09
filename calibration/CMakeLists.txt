foreach(target calibrate_camera calibrate_handeye calibrate_robotworld_handeye)
  add_executable(${target} ${target}.cpp)
  target_link_libraries(${target} PRIVATE tools)
  set_target_properties(${target} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR})
endforeach()

if(BUILD_TESTING)
  add_executable(board_calibration_test ${PROJECT_SOURCE_DIR}/tests/calibration/board_calibration_test.cpp)
  target_link_libraries(board_calibration_test PRIVATE tools)
  foreach(board calibration calibration_chessboard_11x8)
    add_test(NAME calibrate_${board} COMMAND board_calibration_test $<TARGET_FILE:calibrate_camera>
      ${PROJECT_SOURCE_DIR}/configs/${board}.yaml ${PROJECT_BINARY_DIR}/calibration_test/${board})
    set_tests_properties(calibrate_${board} PROPERTIES TIMEOUT 120)
  endforeach()
endif()

if(BUILD_TESTING)
  add_executable(handeye_calibration_test ${PROJECT_SOURCE_DIR}/tests/calibration/handeye_calibration_test.cpp)
  target_link_libraries(handeye_calibration_test PRIVATE tools)
  foreach(solver calibrate_robotworld_handeye calibrate_handeye)
    foreach(board calibration calibration_chessboard_11x8)
      add_test(NAME ${solver}_${board} COMMAND handeye_calibration_test $<TARGET_FILE:${solver}>
        ${PROJECT_SOURCE_DIR}/configs/${board}.yaml ${PROJECT_SOURCE_DIR}/configs/hero.yaml
        ${PROJECT_BINARY_DIR}/calibration_test/${solver}_${board})
      set_tests_properties(${solver}_${board} PROPERTIES TIMEOUT 120)
    endforeach()
  endforeach()
endif()

# 采图需要相机与英雄串口，只在完整构建时提供
if(BUILD_HERO)
  add_executable(capture capture.cpp)
  target_link_libraries(capture PRIVATE io hero_io tools)
  target_compile_definitions(capture PRIVATE CALIB_SOURCE_ROOT="${PROJECT_SOURCE_DIR}")
  set_target_properties(capture PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR})
endif()
