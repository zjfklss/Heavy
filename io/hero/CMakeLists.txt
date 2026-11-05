add_library(hero_io STATIC config.cpp hero_serial.cpp)
target_link_libraries(hero_io PUBLIC tools)
