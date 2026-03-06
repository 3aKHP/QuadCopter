# src/core/CMakeLists.txt
add_library(fc_core STATIC
    pid_controller.cpp
    ahrs.cpp
    motor_mixer.cpp
    attitude_controller.cpp
    altitude_controller.cpp
    flight_controller.cpp
)

target_include_directories(fc_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(fc_core PUBLIC fc_hal)
