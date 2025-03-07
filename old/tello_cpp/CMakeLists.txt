cmake_minimum_required(VERSION 3.10)
project(tello_record)

set(CMAKE_CXX_STANDARD 17)

find_package(OpenCV REQUIRED)
find_package(spdlog REQUIRED)

# Add ctello include directories
include_directories(/usr/local/include)  # if you installed ctello with make install
# Or use the direct path to ctello headers:
# include_directories(${CMAKE_SOURCE_DIR}/../ctello/include)

add_executable(tello_record src/record.cpp)
target_link_libraries(tello_record 
    PRIVATE 
    ctello
    ${OpenCV_LIBS}
    spdlog::spdlog
)

# Add the library search path if needed
link_directories(/usr/local/lib)