#pragma once

#include <array>
#include <chrono>
#include <string>
#include <netinet/in.h>

// Structure to store VRPN tracker data
struct VrpnData {
    std::array<float, 3> position;  // x, y, z
    std::array<float, 4> rotation;  // qw, qx, qy, qz (quaternion)
    std::chrono::steady_clock::time_point timestamp;
};

// Structure to store drone configuration
struct DroneConfig {
    std::string name;
    std::string ip;
    std::string tracker_id;
    int tracker_num;
    int command_socket{-1};
    struct sockaddr_in command_addr{};
    bool is_connected{false};
};

// Structure to hold command and response data
struct CommandData {
    std::string ip;
    std::string command;
    std::string response;
    bool completed{false};
    std::chrono::steady_clock::time_point timestamp;
};