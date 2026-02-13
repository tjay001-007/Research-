cmake_minimum_required(VERSION 3.14)
project(px4_att_control
    VERSION 1.0.0
    DESCRIPTION "PX4 Fixed-Wing Attitude Control Library"
    LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Header-only library target
add_library(px4_att_control INTERFACE)
target_include_directories(px4_att_control INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Examples
option(PX4_ATT_CTRL_BUILD_EXAMPLES "Build example programs" ON)
if(PX4_ATT_CTRL_BUILD_EXAMPLES)
    add_executable(pitch_unstable_sim examples/pitch_unstable_sim.cpp)
    target_link_libraries(pitch_unstable_sim PRIVATE px4_att_control)

    add_executable(step_response examples/step_response.cpp)
    target_link_libraries(step_response PRIVATE px4_att_control)
endif()
