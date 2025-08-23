cmake_minimum_required(VERSION 3.10)

# Project name and version
project(QuaternionFilter VERSION 1.0)

# Set C standard
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

# Add include directories
include_directories(src)

# Create a library from source files
add_library(quaternion_lib 
    src/quaternion.c
    src/complementary_filter.c
)

# Link math library (for sqrt, fabsf, etc.)
target_link_libraries(quaternion_lib m)

# Create executable from test file
add_executable(test1 test1.c)

# Link the library to the executable
target_link_libraries(test1 quaternion_lib m)

# Optional: Add compiler flags for debugging and warnings
target_compile_options(quaternion_lib PRIVATE -Wall -Wextra -g)
target_compile_options(test1 PRIVATE -Wall -Wextra -g)

# Optional: Print build information
message(STATUS "Building quaternion filter library and test")
message(STATUS "C Compiler: ${CMAKE_C_COMPILER}")
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
