cmake_minimum_required(VERSION 3.10)
project(fw_pitch_controller C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

# Compiler warnings
add_compile_options(-Wall -Wextra -Wpedantic -Wshadow)

# --- Main library ---
add_library(fw_pitch_controller STATIC
    src/fw_pitch_rate_ctrl.c
    src/fw_pitch_att_ctrl.c
    src/fw_pitch_tecs.c
    src/fw_pitch_controller.c
)

target_include_directories(fw_pitch_controller PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(fw_pitch_controller PRIVATE m)

# --- Unit tests ---
option(BUILD_TESTS "Build unit tests" ON)

if(BUILD_TESTS)
    enable_testing()

    add_executable(test_pitch_controller test/test_pitch_controller.c)
    target_link_libraries(test_pitch_controller PRIVATE fw_pitch_controller m)
    target_include_directories(test_pitch_controller PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
    )
    add_test(NAME pitch_controller_tests COMMAND test_pitch_controller)
endif()
