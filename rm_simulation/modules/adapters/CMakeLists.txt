# Profile descriptions are standard-library-only and shared by UI and the wire adapter.
add_library(rm_sim_adapter_profiles src/profile.cpp
  algorithms/generic/profile.cpp algorithms/sp_vision/profile.cpp)
target_include_directories(rm_sim_adapter_profiles PUBLIC include)
target_compile_features(rm_sim_adapter_profiles PUBLIC cxx_std_17)
rm_sim_enable_warnings(rm_sim_adapter_profiles)

if(RM_SIM_BUILD_CLI)
  add_subdirectory(sdk/cpp)
  add_library(rm_sim_adapters src/external.cpp src/live.cpp)
  target_include_directories(rm_sim_adapters PUBLIC include)
  target_link_libraries(rm_sim_adapters PUBLIC rm_sim_runner rm_sim_adapter_wire)
  rm_sim_enable_warnings(rm_sim_adapters)
  add_executable(rm_sim_adapter_peer examples/protocol_peer.cpp)
  target_link_libraries(rm_sim_adapter_peer PRIVATE rm_sim_adapter_wire)
endif()
