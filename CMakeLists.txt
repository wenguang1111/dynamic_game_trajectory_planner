cmake_minimum_required(VERSION 3.19.0)
project(dynamic_game_trajectory_planner VERSION 0.1.0 LANGUAGES C CXX ISPC)

enable_language(ISPC)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(ISPC_TARGET "avx2-i32x8")  
set(ispc_files
    src/update_trajectory.ispc
)
set(ISPC_PATH "src/update_trajectory.ispc")
set(ISPC_OUT_O "${CMAKE_CURRENT_BINARY_DIR}/update_trajectory_ispc.o")
set(ISPC_OUT_H "${CMAKE_CURRENT_BINARY_DIR}/update_trajectory_ispc.h")

add_custom_command(
    OUTPUT ${ISPC_OUT_O} ${ISPC_OUT_H}
    COMMAND ispc ${ISPC_PATH} -o ${ISPC_OUT_O} --target=${ISPC_TARGET} --header-outfile=${ISPC_OUT_H}
    DEPENDS ${ISPC_PATH}
    COMMENT "Compiling ISPC: ${ISPC_PATH}"
)

include_directories(include)

set(source_files
    src/main.cpp
    src/dynamic_game_planner.cpp
    src/utils.cpp
    src/vehicle_state.cpp
)

add_executable(dynamic_game_trajectory_planner ${source_files} ${ispc_files})

set(ispc_files
    src/integrate_opt.ispc
)
set_target_properties(dynamic_game_trajectory_planner PROPERTIES
    ISPC_INSTRUCTION_SETS ${ISPC_TARGET}
)

target_include_directories(dynamic_game_trajectory_planner PRIVATE ${CMAKE_CURRENT_BINARY_DIR})