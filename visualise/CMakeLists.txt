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

# Create executable from test file
add_executable(test1 test1.c)

# Create executable for simulated quaternion streaming
add_executable(quaternion_stream dummy_stream.c)

# Create executable for real IMU data streaming
add_executable(real_imu_stream real_imu_stream.c)

# Link the library to executables
target_link_libraries(test1 quaternion_lib m)
target_link_libraries(quaternion_stream quaternion_lib m)
target_link_libraries(real_imu_stream quaternion_lib m)

# Add this to your existing CMakeLists.txt
add_library(quaternion_shared SHARED
    ../src/quaternion.c
    ../src/complementary_filter.c
)
target_link_libraries(quaternion_shared m)
target_compile_options(quaternion_shared PRIVATE -Wall -Wextra -g -fPIC)


# Optional: Add compiler flags for debugging and warnings
target_compile_options(quaternion_lib PRIVATE -Wall -Wextra -g)
target_compile_options(test1 PRIVATE -Wall -Wextra -g)
target_compile_options(quaternion_stream PRIVATE -Wall -Wextra -g)
target_compile_options(real_imu_stream PRIVATE -Wall -Wextra -g)

# Optional: Print build information
message(STATUS "Building quaternion filter library and executables")
message(STATUS "C Compiler: ${CMAKE_C_COMPILER}")
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
