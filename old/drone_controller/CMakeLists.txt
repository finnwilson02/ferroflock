cmake_minimum_required(VERSION 3.10)
project(drone_controller)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find required packages
find_package(OpenCV REQUIRED)
find_package(nlohmann_json 3.2.0 QUIET)
if(NOT nlohmann_json_FOUND)
    include(FetchContent)
    FetchContent_Declare(json URL https://github.com/nlohmann/json/releases/download/v3.11.2/json.tar.xz)
    FetchContent_MakeAvailable(json)
endif()

# Add executable
add_executable(drone_controller drone_controller.cpp)

# Include directories
target_include_directories(drone_controller PRIVATE ${OpenCV_INCLUDE_DIRS})

# Link libraries
target_link_libraries(drone_controller PRIVATE ${OpenCV_LIBS})

if(nlohmann_json_FOUND)
    target_link_libraries(drone_controller PRIVATE nlohmann_json::nlohmann_json)
else()
    target_link_libraries(drone_controller PRIVATE nlohmann_json::nlohmann_json)
endif()

# Install target
install(TARGETS drone_controller DESTINATION bin)