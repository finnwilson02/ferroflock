cmake_minimum_required(VERSION 3.10)
project(helper_functions)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find OpenCV
find_package(OpenCV REQUIRED)

# Include directories
include_directories(
    ${OpenCV_INCLUDE_DIRS}
    /usr/include/opencv4
    ../libs/vrpn
    include
    .
)
include_directories(${CMAKE_SOURCE_DIR}/include)

# Define source files for each library
set(TELLO_CONTROLLER_SOURCES
    src/tello_controller.cpp
)

set(TELLO_IMU_HANDLER_SOURCES
    src/tello_imu_handler.cpp
)

set(LOGGER_SOURCES
    src/logger.cpp
)

set(OPTITRACK_SOURCES
    src/optitrack.cpp
)

set(CALIBRATION_SOURCES
    src/calibration.cpp
)

set(KEYBOARD_CONTROL_SOURCES
    src/keyboard_control.cpp
)

set(MAP_OPTITRACK_SOURCES
    src/map_optitrack.cpp
)

set(MENU_SOURCES
    src/menu.cpp
)

set(MAIN_SOURCES
    src/main.cpp
)

set(TEST_OPTITRACK_LOGGING_SOURCES
    tests/test_optitrack_logging.cpp
)

set(TEST_IMU_SOURCES
    tests/test_imu.cpp
)

set(TEST_KEYBOARD_CONTROL_SOURCES
    tests/test_keyboard_control.cpp
)

# Create libraries
add_library(tello_controller ${TELLO_CONTROLLER_SOURCES})
add_library(tello_imu_handler ${TELLO_IMU_HANDLER_SOURCES})
add_library(logger ${LOGGER_SOURCES})
add_library(optitrack ${OPTITRACK_SOURCES})
add_library(calibration ${CALIBRATION_SOURCES})
add_library(keyboard_control ${KEYBOARD_CONTROL_SOURCES})
add_library(map_optitrack ${MAP_OPTITRACK_SOURCES})
add_library(menu ${MENU_SOURCES})

# Create main executable
add_executable(helper_functions ${MAIN_SOURCES})

# Create test executables
add_executable(test_optitrack_logging ${TEST_OPTITRACK_LOGGING_SOURCES})
add_executable(test_imu ${TEST_IMU_SOURCES})
add_executable(test_keyboard_control ${TEST_KEYBOARD_CONTROL_SOURCES})

# Set dependencies for libraries
target_link_libraries(tello_imu_handler PRIVATE tello_controller)
target_link_libraries(optitrack PRIVATE logger ${OpenCV_LIBS} vrpn quat)
target_link_libraries(calibration PRIVATE logger optitrack tello_controller)
target_link_libraries(keyboard_control PRIVATE logger tello_controller)
target_link_libraries(map_optitrack PRIVATE logger optitrack tello_controller keyboard_control)
target_link_libraries(menu PRIVATE logger optitrack tello_controller calibration keyboard_control map_optitrack)

# Link libraries to main executable
target_link_libraries(helper_functions
    PRIVATE
    tello_controller
    tello_imu_handler
    logger
    optitrack
    calibration
    keyboard_control
    map_optitrack
    menu
    ${OpenCV_LIBS}
    vrpn
    quat
    pthread
)

# Link libraries to test executables
target_link_libraries(test_optitrack_logging
    PRIVATE
    logger
    optitrack
    ${OpenCV_LIBS}
    vrpn
    quat
    pthread
)

target_link_libraries(test_imu
    PRIVATE
    tello_controller
    tello_imu_handler
    logger
    menu
    optitrack
    ${OpenCV_LIBS}
    vrpn
    quat
    pthread
)

# Link libraries to test_keyboard_control
target_link_libraries(test_keyboard_control
    PRIVATE
    tello_controller
    logger
    keyboard_control
    pthread
)

# Install targets
install(TARGETS helper_functions test_optitrack_logging test_imu test_keyboard_control DESTINATION bin)

# No need to copy configuration files to the build directory
# Configuration files will be accessed relative to the executable