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

# Define source files for each library
set(TELLO_CONTROLLER_SOURCES
    src/tello_controller.cpp
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

set(MENU_SOURCES
    src/menu.cpp
)

set(MAIN_SOURCES
    src/main.cpp
)

# Create libraries
add_library(tello_controller ${TELLO_CONTROLLER_SOURCES})
add_library(logger ${LOGGER_SOURCES})
add_library(optitrack ${OPTITRACK_SOURCES})
add_library(calibration ${CALIBRATION_SOURCES})
add_library(menu ${MENU_SOURCES})

# Create main executable
add_executable(helper_functions ${MAIN_SOURCES})

# Set dependencies for libraries
target_link_libraries(optitrack PRIVATE logger ${OpenCV_LIBS} vrpn quat)
target_link_libraries(calibration PRIVATE logger optitrack tello_controller)
target_link_libraries(menu PRIVATE logger optitrack tello_controller calibration)

# Link libraries to main executable
target_link_libraries(helper_functions
    PRIVATE
    tello_controller
    logger
    optitrack
    calibration
    menu
    ${OpenCV_LIBS}
    vrpn
    quat
    pthread
)

# Install targets
install(TARGETS helper_functions DESTINATION bin)

# No need to copy configuration files to the build directory
# Configuration files will be accessed relative to the executable