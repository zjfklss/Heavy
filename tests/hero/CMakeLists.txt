add_executable(hero_test hero_test.cpp)
target_link_libraries(hero_test PRIVATE auto_outpost hero_io util)

add_test(NAME hero_unit COMMAND hero_test ${PROJECT_SOURCE_DIR}/configs/hero.yaml)
add_test(NAME hero_config COMMAND hero ${PROJECT_SOURCE_DIR}/configs/hero.yaml --check-config)
add_test(NAME hero_smoke COMMAND hero ${PROJECT_SOURCE_DIR}/configs/hero.yaml --smoke-test)
set_tests_properties(hero_smoke PROPERTIES TIMEOUT 90)
