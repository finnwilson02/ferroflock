cmake_minimum_required(VERSION 3.10)
project(swarm)
set(CMAKE_CXX_STANDARD 17)

# Find required packages
find_package(OpenCV REQUIRED)
find_package(spdlog REQUIRED)

# Add ctello include directories
include_directories(/usr/local/include)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(swarm src/swarm.cpp)

target_link_libraries(swarm
    PRIVATE
    ctello
    ${OpenCV_LIBS}
    spdlog::spdlog
)

# Add the library search path if needed
link_directories(/usr/local/lib)