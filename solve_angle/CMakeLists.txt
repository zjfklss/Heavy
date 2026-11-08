cmake_minimum_required(VERSION 3.10)
project(solve_angle)

## Use C++17 (for std::optional, etc.)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#######################
## Find dependencies ##
#######################

find_package(ament_cmake_auto REQUIRED)
find_package(Eigen3 REQUIRED)
ament_auto_find_build_dependencies()

#######################
## TinyMPC library   ##
#######################

set(TINYMPC_SRC
    tinympc/admm.cpp
    tinympc/tiny_api.cpp
    tinympc/codegen.cpp
    tinympc/rho_benchmark.cpp
)

add_library(tinympc STATIC ${TINYMPC_SRC})
target_include_directories(tinympc PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${EIGEN3_INCLUDE_DIR}
)
set_property(TARGET tinympc PROPERTY POSITION_INDEPENDENT_CODE ON)

###########
## Build ##
###########

ament_auto_add_library(new_fire_lib SHARED
    src/opti_solver.cpp
    src/ballistic_solver.cpp
    src/planner.cpp
)
target_include_directories(new_fire_lib PUBLIC include)
target_link_libraries(new_fire_lib tinympc)
target_include_directories(new_fire_lib PUBLIC ${EIGEN3_INCLUDE_DIR})

ament_auto_add_library(${PROJECT_NAME} SHARED
    src/opti_solver.cpp
    src/ballistic_solver.cpp
    src/planner.cpp
)
target_link_libraries(${PROJECT_NAME} tinympc)
target_include_directories(${PROJECT_NAME} PUBLIC ${EIGEN3_INCLUDE_DIR})

rclcpp_components_register_node(${PROJECT_NAME}
    PLUGIN solve_angle::SolveAngle
    EXECUTABLE ${PROJECT_NAME}_node
)

#############
## Testing ##
#############

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    list(APPEND AMENT_LINT_AUTO_EXCLUDE
        ament_cmake_copyright
        ament_cmake_uncrustify
    )
    ament_lint_auto_find_test_dependencies()

    find_package(ament_cmake_gtest REQUIRED)
    find_package(rclcpp REQUIRED)
    find_package(jlcv_interfaces REQUIRED)
    find_package(geometry_msgs REQUIRED)

    # --- 测试 1: 弹道解算 ---
    ament_add_gtest(test_ballistic test/test_ballistic.cpp)
    target_link_libraries(test_ballistic new_fire_lib)
    target_include_directories(test_ballistic PUBLIC include)

    # --- 测试 2: 前哨站逻辑 ---
    ament_add_gtest(test_outpost test/test_outpost.cpp)
    target_link_libraries(test_outpost new_fire_lib)
    ament_target_dependencies(test_outpost
        rclcpp
        jlcv_interfaces
        geometry_msgs
    )
    target_include_directories(test_outpost PUBLIC include)

endif()

#############
## Install ##
#############

ament_auto_package()
