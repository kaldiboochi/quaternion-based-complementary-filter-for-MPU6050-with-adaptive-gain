cmake_minimum_required(VERSION 3.10)

# Project name and version
project(QuaternionFilter VERSION 1.0)

# Set C standard
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

# Add include directories
include_directories(../src)

# Create a library from source files
add_library(quaternion_lib 
    ../src/quaternion.c
    ../src/complementary_filter.c
)

# Link math library (for sqrt, fabsf, etc.)
target_link_libraries(quaternion_lib m)


# Add this to your existing CMakeLists.txt
add_library(quaternion_shared SHARED
    ../src/quaternion.c
    ../src/complementary_filter.c
)
target_link_libraries(quaternion_shared m)
target_compile_options(quaternion_shared PRIVATE -Wall -Wextra -g -fPIC)


# Optional: Add compiler flags for debugging and warnings
target_compile_options(quaternion_lib PRIVATE -Wall -Wextra -g)

# Optional: Print build information
message(STATUS "Building quaternion filter library and executables")
message(STATUS "C Compiler: ${CMAKE_C_COMPILER}")
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
