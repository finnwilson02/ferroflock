cmake_minimum_required(VERSION 3.10)
project(frame_calibration)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find OpenCV
find_package(OpenCV REQUIRED)

# Include directories
include_directories(
    ${OpenCV_INCLUDE_DIRS}
    /usr/include/opencv4
    ../libs/vrpn
    .
)

# No need for nlohmann_json anymore, using custom JSON implementation

# Create executables
add_executable(device_scanner src/device_scanner.cpp)
add_executable(frame_calibration frame_calibration.cpp)
add_executable(drone_control src/drone_control.cpp src/optitrack_viz.cpp)

# Link libraries for device_scanner
target_link_libraries(device_scanner)

# Link libraries for frame_calibration
target_link_libraries(frame_calibration
    PRIVATE
    ${OpenCV_LIBS}
    vrpn
    quat
    pthread
)

# Link libraries for drone_control
target_link_libraries(drone_control
    PRIVATE
    ${OpenCV_LIBS}
    vrpn
    quat
    pthread
)

# No longer using nlohmann_json

# Install targets
install(TARGETS device_scanner frame_calibration drone_control DESTINATION bin)

# Copy Python scripts and configuration files
file(COPY python/get_devices.py DESTINATION ${CMAKE_BINARY_DIR}/python)
file(COPY dji_devices.json DESTINATION ${CMAKE_BINARY_DIR})