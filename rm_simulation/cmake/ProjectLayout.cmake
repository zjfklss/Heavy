# Build files are grouped under build/; work artifacts stay outside the source tree.
function(rm_sim_require_external_directory path description)
  get_filename_component(resolved "${path}" REALPATH BASE_DIR "${CMAKE_BINARY_DIR}")
  file(RELATIVE_PATH relative "${CMAKE_CURRENT_SOURCE_DIR}" "${resolved}")
  if(NOT IS_ABSOLUTE "${relative}" AND NOT relative MATCHES "^\\.\\.(/|$)")
    message(FATAL_ERROR
      "${description} must be outside the source tree. "
      "Use ../rm_simulation-artifacts for generated files.")
  endif()
endfunction()

get_filename_component(build_directory "${CMAKE_BINARY_DIR}" REALPATH)
file(RELATIVE_PATH build_relative "${CMAKE_CURRENT_SOURCE_DIR}" "${build_directory}")
if(NOT IS_ABSOLUTE "${build_relative}" AND
   NOT build_relative MATCHES "^\\.\\.(/|$)" AND
   NOT build_relative MATCHES "^build(/|$)")
  message(FATAL_ERROR
    "Build directory must be under build/ or outside the source tree. "
    "Use build/<configuration>; do not build in the source root or module directories.")
endif()
set(RM_SIM_ARTIFACT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../rm_simulation-artifacts"
  CACHE PATH "Directory for generated traces, screenshots and test artifacts")
get_filename_component(RM_SIM_ARTIFACT_DIR "${RM_SIM_ARTIFACT_DIR}" ABSOLUTE)
rm_sim_require_external_directory("${RM_SIM_ARTIFACT_DIR}" "Artifact directory")

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")

function(rm_sim_enable_warnings target)
  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic)
  elseif(MSVC)
    target_compile_options(${target} PRIVATE /W4)
  endif()
endfunction()
