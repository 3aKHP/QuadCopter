# src/CMakeLists.txt
add_subdirectory(core)
add_subdirectory(hal)

if(BUILD_SITL)
    add_subdirectory(sitl)
endif()
