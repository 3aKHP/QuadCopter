# src/sitl/CMakeLists.txt

add_library(fc_sitl_lib STATIC
    physics_model.cpp
    sitl_imu.cpp
    sitl_barometer.cpp
    sitl_motor.cpp
    sitl_timer.cpp
    sitl_logger.cpp
)

target_include_directories(fc_sitl_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(fc_sitl_lib PUBLIC fc_core fc_hal)

# SITL executable
add_executable(quadcopter_sitl sitl_main.cpp)
target_link_libraries(quadcopter_sitl PRIVATE fc_sitl_lib fc_core)
