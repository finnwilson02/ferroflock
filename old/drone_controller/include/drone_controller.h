#pragma once

#include <iostream>
#include <vector>
#include <chrono>
#include <thread>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <cstring>
#include <cmath>
#include <mutex>
#include <atomic>
#include <string>
#include <unordered_map>
#include <memory>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <filesystem>
#include <signal.h>
#include <sys/wait.h>
#include <fcntl.h>
#include <poll.h>
#include <deque>

#include "drone_types.h"
#include "vrpn_client.h"

// Constants for flight control
const int MOVE_UP_DURATION_MS = 2000;    // 2 seconds moving up
const int MOVE_FORWARD_DURATION_MS = 5000; // 5 seconds moving forward
const int TAKEOFF_WAIT_MS = 5000;        // Wait 5 seconds for takeoff
const int UP_SPEED = 30;                 // Not max speed (0-100)
const int FORWARD_SPEED = 30;            // Not max speed (0-100)
const int MAX_PATH_POINTS = 500;         // Maximum number of points to store

// Class for drone controller and visualization
class DroneController {
private:
    std::vector<DroneConfig> drones;
    std::unordered_map<std::string, int> drone_sockets;
    std::unordered_map<std::string, sockaddr_in> drone_addresses;
    
    std::atomic<bool> running{true};
    std::atomic<bool> visualization_active{false};
    
    std::mutex tracker_mutex;
    std::unordered_map<std::string, VrpnData> tracker_data;
    std::atomic<bool> has_vrpn_data{false};
    
    pid_t vrpn_viz_pid{-1};
    int vrpn_socket{-1};
    
    int selected_drone_index{-1};
    
    // Path tracing
    std::unordered_map<std::string, std::deque<cv::Point2f>> drone_paths;
    
    // Transform from OptiTrack to command frame
    float yaw_transform{0.0f};
    
    // Visualization window
    cv::Mat display;
    std::string window_name{"Drone Controller"};
    
    // Visualization settings
    const double SCALE = 100.0;  // Scale factor for visualization
    const double OFFSET_X = 0.0;  // Offset to center the visualization
    const double OFFSET_Y = 0.0;

    // Private methods
    bool initSocket(DroneConfig& drone);
    std::string sendCommandBlocking(const std::string& ip, const std::string& cmd, int timeout_ms = 2000);
    float quaternionToYaw(const std::array<float, 4>& q);
    cv::Point worldToScreen(float x, float y);
    void drawGrid();
    void drawOrientationArrow(const std::string& tracker_id, const cv::Point& pos, 
                              const std::array<float, 4>& quaternion, const cv::Scalar& color);
    void drawDronePaths();
    void updateVisualization();
    pid_t launchVrpnViz();
    void calculateYawTransform(const std::string& tracker_id, float forward_direction);
    void showAvailableDrones();

public:
    DroneController();
    ~DroneController();
    
    void loadDroneConfigs();
    void initializeDrones();
    bool connectVrpn();
    bool selectDrone();
    void executeFlightSequence();
    void rebootDrone();
    void showMenu();
    void run();
};