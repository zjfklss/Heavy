add_library(hero_simulation_launcher STATIC launcher.cpp)
target_link_libraries(hero_simulation_launcher PUBLIC nlohmann_json::nlohmann_json Threads::Threads)
target_compile_definitions(hero_simulation_launcher PRIVATE HERO_SOURCE_ROOT="${PROJECT_SOURCE_DIR}")

if(HERO_WITH_SIMULATION)
  add_library(hero_simulation STATIC runner.cpp)
  target_link_libraries(hero_simulation PUBLIC
    hero_simulation_io hero_simulation_launcher jlcv_show_model auto_outpost auto_aim tools)
endif()
