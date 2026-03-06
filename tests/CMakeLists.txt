# tests/CMakeLists.txt

include(FetchContent)
FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG        v1.14.0
)
# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

# Test executables
add_executable(test_pid test_pid.cpp)
target_link_libraries(test_pid PRIVATE fc_core GTest::gtest_main)
add_test(NAME test_pid COMMAND test_pid)

add_executable(test_ahrs test_ahrs.cpp)
target_link_libraries(test_ahrs PRIVATE fc_core GTest::gtest_main)
add_test(NAME test_ahrs COMMAND test_ahrs)

add_executable(test_motor_mixer test_motor_mixer.cpp)
target_link_libraries(test_motor_mixer PRIVATE fc_core GTest::gtest_main)
add_test(NAME test_motor_mixer COMMAND test_motor_mixer)

add_executable(test_attitude_ctrl test_attitude_ctrl.cpp)
target_link_libraries(test_attitude_ctrl PRIVATE fc_core fc_sitl_lib GTest::gtest_main)
add_test(NAME test_attitude_ctrl COMMAND test_attitude_ctrl)

add_executable(test_altitude_ctrl test_altitude_ctrl.cpp)
target_link_libraries(test_altitude_ctrl PRIVATE fc_core fc_sitl_lib GTest::gtest_main)
add_test(NAME test_altitude_ctrl COMMAND test_altitude_ctrl)
