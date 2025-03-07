# CMakeLists.txt
cmake_minimum_required(VERSION 3.10)
project(drift_calibrator)

set(CMAKE_CXX_STANDARD 17)

# Add source files
add_executable(drift_calibrator 
    src/drift_calibrator.cpp
    src/file_writer.cpp
)

# Create directories if they don't exist
file(MAKE_DIRECTORY ${CMAKE_SOURCE_DIR}/src)
file(MAKE_DIRECTORY ${CMAKE_SOURCE_DIR}/include)

# Include directories
target_include_directories(drift_calibrator
    PRIVATE
    ${CMAKE_SOURCE_DIR}/include
)