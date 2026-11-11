add_executable(rm_sim_image_test integration/image_test.cpp)
target_link_libraries(rm_sim_image_test PRIVATE rm_sim_runner)
add_test(NAME rm_sim_image COMMAND rm_sim_image_test)
add_test(NAME rm_sim_image_cpu COMMAND rm_sim_image_test cpu)
option(RM_SIM_TEST_GPU "Run camera regression tests requiring a hardware EGL GPU" OFF)
if(RM_SIM_TEST_GPU)
  add_test(NAME rm_sim_image_gpu COMMAND rm_sim_image_test gpu)
endif()
add_executable(rm_sim_loop_test integration/loop_test.cpp)
target_link_libraries(rm_sim_loop_test PRIVATE rm_sim_runner)
add_test(NAME rm_sim_loop COMMAND rm_sim_loop_test)

# Test images/traces have their own external artifact area, separate from binaries.
get_filename_component(build_name "${CMAKE_BINARY_DIR}" NAME)
set(test_artifacts "${RM_SIM_ARTIFACT_DIR}/tests/${build_name}")
file(MAKE_DIRECTORY "${test_artifacts}")
if(RM_SIM_BUILD_CLI)
  find_package(Python3 COMPONENTS Interpreter REQUIRED)
  set(scenarios "${PROJECT_SOURCE_DIR}/configs/scenarios")
  set(adapter_test_args $<TARGET_FILE:rm_sim_loop> $<TARGET_FILE:rm_sim_adapter_peer>)
  if(RM_SIM_BUILD_QT)
    add_test(NAME rm_sim_live COMMAND ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/protocol/live_test.py $<TARGET_FILE:rm_sim_loop>
      $<TARGET_FILE:rm_sim_qt> ${test_artifacts}/live)
    set_tests_properties(rm_sim_live PROPERTIES TIMEOUT 30)
    add_test(NAME rm_sim_live_reconnect COMMAND ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/protocol/live_reconnect_test.py $<TARGET_FILE:rm_sim_loop>
      $<TARGET_FILE:rm_sim_qt> ${test_artifacts}/live-reconnect)
    set_tests_properties(rm_sim_live_reconnect PROPERTIES TIMEOUT 30)
    list(APPEND adapter_test_args $<TARGET_FILE:rm_sim_qt> ${test_artifacts}/adapter_ui_preview.png)
  endif()
  add_test(NAME rm_sim_adapters_protocol COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/protocol/adapters_test.py ${adapter_test_args})
  set_tests_properties(rm_sim_adapters_protocol PROPERTIES TIMEOUT 30)
  add_test(NAME rm_sim_scenarios COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/protocol/scenario_test.py $<TARGET_FILE:rm_sim> ${scenarios})
  add_test(NAME rm_sim_image_protocol COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/protocol/image_protocol_test.py $<TARGET_FILE:rm_sim_loop>)
  add_test(NAME rm_sim_loop_protocol COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/protocol/loop_protocol_test.py $<TARGET_FILE:rm_sim_loop> ${scenarios}/closed_loop)
  if(RM_SIM_BUILD_QT)
    add_test(NAME rm_sim_camera_trace COMMAND rm_sim_loop
      ${scenarios}/camera/image.json ${test_artifacts}/camera_test_trace.json)
    set_tests_properties(rm_sim_camera_trace PROPERTIES FIXTURES_SETUP camera_trace TIMEOUT 30)
    add_test(NAME rm_sim_camera_qt COMMAND rm_sim_qt ${test_artifacts}/camera_test_trace.json
      --self-test --screenshot ${test_artifacts}/camera_test_preview.png)
    set_tests_properties(rm_sim_camera_qt PROPERTIES FIXTURES_REQUIRED camera_trace
      ENVIRONMENT "QT_QPA_PLATFORM=offscreen" TIMEOUT 20)
    add_test(NAME rm_sim_loop_trace COMMAND rm_sim_loop
      ${scenarios}/closed_loop/ideal.json ${test_artifacts}/loop_test_trace.json)
    set_tests_properties(rm_sim_loop_trace PROPERTIES FIXTURES_SETUP loop_trace)
    add_test(NAME rm_sim_loop_qt COMMAND rm_sim_qt ${test_artifacts}/loop_test_trace.json --self-test)
    set_tests_properties(rm_sim_loop_qt PROPERTIES FIXTURES_REQUIRED loop_trace
      ENVIRONMENT "QT_QPA_PLATFORM=offscreen" TIMEOUT 20)
    add_test(NAME rm_sim_qt_trace COMMAND rm_sim
      ${scenarios}/world_demo.json ${test_artifacts}/qt_test_trace.json)
    set_tests_properties(rm_sim_qt_trace PROPERTIES FIXTURES_SETUP qt_trace)
    add_test(NAME rm_sim_qt_ui COMMAND rm_sim_qt ${test_artifacts}/qt_test_trace.json
      --self-test --screenshot ${test_artifacts}/qt_test_preview.png)
    set_tests_properties(rm_sim_qt_ui PROPERTIES FIXTURES_REQUIRED qt_trace
      ENVIRONMENT "QT_QPA_PLATFORM=offscreen" TIMEOUT 20)
    add_test(NAME rm_sim_qt_loop_config COMMAND rm_sim_qt --self-test-loop-config
      --screenshot ${test_artifacts}/loop_config_preview.png)
    set_tests_properties(rm_sim_qt_loop_config PROPERTIES
      ENVIRONMENT "QT_QPA_PLATFORM=offscreen" TIMEOUT 60)
    add_test(NAME rm_sim_qt_config COMMAND rm_sim_qt --self-test-config
      --screenshot ${test_artifacts}/config_test_preview.png)
    set_tests_properties(rm_sim_qt_config PROPERTIES
      ENVIRONMENT "QT_QPA_PLATFORM=offscreen" TIMEOUT 60)
  endif()
endif()
