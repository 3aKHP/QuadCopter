cmake_minimum_required(VERSION 3.16)
project(QuadCopter VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Platform-specific definitions
if(MSVC)
    add_definitions(-D_USE_MATH_DEFINES)
    add_compile_options(/utf-8)
endif()

# Options
option(BUILD_SITL "Build SITL simulation executable" ON)
option(BUILD_TESTS "Build unit tests" ON)

# Core flight controller library (platform-independent)
add_subdirectory(src)

# Unit tests
if(BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()
