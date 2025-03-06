cmake_minimum_required(VERSION 3.10)
project(vrpn_viz)

set(CMAKE_CXX_STANDARD 17)

# Find OpenCV
find_package(OpenCV REQUIRED)

# Include directories
include_directories(
    ${OpenCV_INCLUDE_DIRS}
    /usr/include/opencv4
    ../libs/vrpn
)

# Add executable
add_executable(vrpn_viz vrpn_viz.cpp)

# Link libraries
target_link_libraries(vrpn_viz
    PRIVATE
    ${OpenCV_LIBS}
    vrpn
    quat
    pthread
)