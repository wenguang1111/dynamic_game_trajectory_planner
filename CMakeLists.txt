cmake_minimum_required(VERSION 3.10.0)
project(dynamic_game_trajectory_planner VERSION 0.1.0 LANGUAGES C CXX)

include_directories(include)

set(source_files
    src/main.cpp
    src/dynamic_game_planner.cpp
    src/utils.cpp
    src/vehicle_state.cpp
)

add_executable(dynamic_game_trajectory_planner ${source_files})

