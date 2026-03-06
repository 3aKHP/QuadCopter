# src/hal/CMakeLists.txt
# HAL is a header-only interface library
add_library(fc_hal INTERFACE)
target_include_directories(fc_hal INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})
