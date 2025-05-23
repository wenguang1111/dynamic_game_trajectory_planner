cmake_minimum_required(VERSION 3.19.0)
project(dynamic_game_trajectory_planner VERSION 0.1.0 LANGUAGES C CXX ISPC)

enable_language(ISPC)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(ISPC_TARGET "avx2-i32x8")
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
# set(ispc_files
#     src/integrate.ispc
# )
set(ISPC_PATH "${CMAKE_SOURCE_DIR}/src/integrate.ispc")
set(ISPC_OUT_O "${CMAKE_CURRENT_BINARY_DIR}/integrate_ispc.o")
set(ISPC_OUT_H "${CMAKE_CURRENT_BINARY_DIR}/integrate_ispc.h")
set(ISPC_OUT_S "${CMAKE_CURRENT_BINARY_DIR}/integrate_ispc.s")

# add_custom_command(
#     OUTPUT ${ISPC_OUT_O} ${ISPC_OUT_H} ${ISPC_OUT_S}
#     COMMAND ispc ${ISPC_PATH} -o ${ISPC_OUT_O} -I${CMAKE_CURRENT_SOURCE_DIR}/include -O3 --target=${ISPC_TARGET} --header-outfile=${ISPC_OUT_H} 
#     DEPENDS ${ISPC_PATH}
#     COMMENT "Compiling ISPC: ${ISPC_PATH}"
# )
add_custom_command(
    OUTPUT  ${ISPC_OUT_H} ${ISPC_OUT_S}
    COMMAND ispc ${ISPC_PATH}
            # -o ${ISPC_OUT_O}
            --emit-asm -g
            -I${CMAKE_CURRENT_SOURCE_DIR}/include
            --target=${ISPC_TARGET}
            --header-outfile=${ISPC_OUT_H}
            -o ${ISPC_OUT_S}
    DEPENDS ${ISPC_PATH}
    COMMENT "Compiling ISPC (with assembly): ${ISPC_PATH}"
)

include_directories(include Recorder4Cpp ISPC_OUT_H)

option(USE_RECORDER "Use recorder" OFF)
set(recorder_file ${recorder_file})

if (USE_RECORDER)
  MESSAGE(STATUS "USE_RECORDER=ON")
  add_definitions(-DUSE_RECORDER)
  set(recorder_file ${recorder_file} Recorder4Cpp/recorder.h
  	Recorder4Cpp/recorder.cpp)
endif()
add_library(integrate_ispc STATIC ${ISPC_OUT_O})
set_target_properties(integrate_ispc PROPERTIES LINKER_LANGUAGE CXX)
target_include_directories(integrate_ispc PUBLIC ${CMAKE_CURRENT_BINARY_DIR})

set(source_files
    src/main.cpp
    src/dynamic_game_planner.cpp
    src/utils.cpp
    src/vehicle_state.cpp
)

# add_executable(dynamic_game_trajectory_planner ${source_files} ${recorder_file})
add_executable(dynamic_game_trajectory_planner ${source_files} ${ISPC_PATH} ${recorder_file})

set_target_properties(dynamic_game_trajectory_planner PROPERTIES
    ISPC_INSTRUCTION_SETS ${ISPC_TARGET}
)

# target_link_libraries(dynamic_game_trajectory_planner PRIVATE update_trajectory_ispc)
target_include_directories(dynamic_game_trajectory_planner PRIVATE ${CMAKE_CURRENT_BINARY_DIR})

# add_custom_command(
#     OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/dynamic_game_planner.s
#     COMMAND ${CMAKE_CXX_COMPILER}
#             -S -g -fverbose-asm -O3
#             -I${CMAKE_CURRENT_SOURCE_DIR}/include
#             -I${CMAKE_CURRENT_SOURCE_DIR}/Recorder4Cpp
#             -o ${CMAKE_CURRENT_BINARY_DIR}/dynamic_game_planner.s
#             ${CMAKE_CURRENT_SOURCE_DIR}/src/dynamic_game_planner.cpp
#     DEPENDS src/dynamic_game_planner.cpp
#     COMMENT "Generating annotated assembly for dynamic_game_planner.cpp"
# )
add_custom_target(generate_ispc_asm
    DEPENDS ${ISPC_OUT_S}
    COMMENT "Generating annotated assembly for ISPC file"
)



