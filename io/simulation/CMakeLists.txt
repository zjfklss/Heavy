add_library(hero_simulation_io STATIC bridge.cpp)
target_link_libraries(hero_simulation_io PUBLIC hero_io rm_sim_adapter_wire ${OpenCV_LIBS})
