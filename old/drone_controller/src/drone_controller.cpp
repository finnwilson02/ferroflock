#include "drone_controller.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <cmath>
#include <algorithm>
#include <nlohmann/json.hpp>

// For convenience
using json = nlohmann::json;

DroneController::DroneController() {
    std::cout << "Starting Drone Controller..." << std::endl;
    
    // Initialize the visualization window
    cv::namedWindow(window_name, cv::WINDOW_AUTOSIZE);
    display = cv::Mat(800, 1000, CV_8UC3, cv::Scalar(0, 0, 0));
    
    // Load drone configurations from JSON
    loadDroneConfigs();
    
    // Initialize drone connections
    initializeDrones();
    
    // Connect to VRPN server for tracking data
    if (connectVrpn()) {
        std::cout << "Connected to VRPN server for tracking data" << std::endl;
    } else {
        std::cout << "Warning: Failed to connect to VRPN server!" << std::endl;
    }
}

DroneController::~DroneController() {
    std::cout << "Shutting down Drone Controller..." << std::endl;
    
    // Kill the visualizer process if it's running
    if (vrpn_viz_pid > 0) {
        kill(vrpn_viz_pid, SIGTERM);
        // Wait for it to exit
        int status;
        waitpid(vrpn_viz_pid, &status, 0);
    }
    
    // Close all sockets
    if (vrpn_socket >= 0) {
        close(vrpn_socket);
    }
    
    for (const auto& [ip, sock] : drone_sockets) {
        if (sock >= 0) {
            close(sock);
        }
    }
    
    // Release any OpenCV windows
    cv::destroyAllWindows();
}

// Load drone configurations from JSON file
void DroneController::loadDroneConfigs() {
    // Since we know the IPs directly, let's just hardcode them instead of using JSON
    // Bird 5 (tracker #8) - 107
    // Bird 3 (tracker #6) - 106
    // Bird 4 (tracker #7) - 104
    // Bird 1 (tracker #4) - 108
    
    // Add Bird 5
    DroneConfig drone1;
    drone1.name = "Bird 5";
    drone1.tracker_id = "Bird5";
    drone1.tracker_num = 8;
    drone1.ip = "192.168.1.107";
    drones.push_back(drone1);
    std::cout << "Added drone: " << drone1.name << " (IP: " << drone1.ip 
              << ", Tracker: " << drone1.tracker_id << ")" << std::endl;
    
    // Add Bird 3
    DroneConfig drone2;
    drone2.name = "Bird 3";
    drone2.tracker_id = "Bird3";
    drone2.tracker_num = 6;
    drone2.ip = "192.168.1.106";
    drones.push_back(drone2);
    std::cout << "Added drone: " << drone2.name << " (IP: " << drone2.ip 
              << ", Tracker: " << drone2.tracker_id << ")" << std::endl;
    
    // Add Bird 4
    DroneConfig drone3;
    drone3.name = "Bird 4";
    drone3.tracker_id = "Bird4";
    drone3.tracker_num = 7;
    drone3.ip = "192.168.1.104";
    drones.push_back(drone3);
    std::cout << "Added drone: " << drone3.name << " (IP: " << drone3.ip 
              << ", Tracker: " << drone3.tracker_id << ")" << std::endl;
    
    // Add Bird 1
    DroneConfig drone4;
    drone4.name = "Bird 1";
    drone4.tracker_id = "Bird1";
    drone4.tracker_num = 4;
    drone4.ip = "192.168.1.108";
    drones.push_back(drone4);
    std::cout << "Added drone: " << drone4.name << " (IP: " << drone4.ip 
              << ", Tracker: " << drone4.tracker_id << ")" << std::endl;
    
    std::cout << "Added " << drones.size() << " drones with hardcoded configuration" << std::endl;
}

// Initialize socket for a drone
bool DroneController::initSocket(DroneConfig& drone) {
    if (drone_sockets.count(drone.ip) > 0 && drone_sockets[drone.ip] >= 0) {
        close(drone_sockets[drone.ip]);
    }

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) {
        std::cerr << "Failed to create socket for " << drone.ip << std::endl;
        return false;
    }

    // Set up local address
    struct sockaddr_in local_addr{};
    local_addr.sin_family = AF_INET;
    local_addr.sin_addr.s_addr = INADDR_ANY;
    local_addr.sin_port = htons(0);  // Let system choose port

    // Bind to local port
    if (bind(sock, (struct sockaddr *)&local_addr, sizeof(local_addr)) < 0) {
        std::cerr << "Failed to bind socket for " << drone.ip << ": " << strerror(errno) << std::endl;
        close(sock);
        return false;
    }

    // Set up remote address
    struct sockaddr_in remote_addr{};
    remote_addr.sin_family = AF_INET;
    remote_addr.sin_addr.s_addr = inet_addr(drone.ip.c_str());
    remote_addr.sin_port = htons(8889);

    // Set socket timeout
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 500000;  // 500ms timeout
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    drone_sockets[drone.ip] = sock;
    drone_addresses[drone.ip] = remote_addr;

    return true;
}

// Initialize all drones by testing connection
void DroneController::initializeDrones() {
    std::cout << "Initializing drones..." << std::endl;
    int connected_count = 0;
    
    for (auto& drone : drones) {
        std::cout << "Connecting to " << drone.ip << "..." << std::endl;
        drone.is_connected = false; // Reset connection status
        
        // Try multiple times to initialize socket
        bool socket_initialized = false;
        for (int attempt = 1; attempt <= 3; attempt++) {
            if (initSocket(drone)) {
                socket_initialized = true;
                break;
            }
            std::cerr << "Failed to initialize socket for " << drone.ip 
                     << " (attempt " << attempt << "/3)" << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
        
        if (!socket_initialized) {
            std::cerr << "Could not initialize socket for " << drone.ip 
                     << " after multiple attempts. Skipping this drone." << std::endl;
            continue;
        }
        
        // Test connection with a command
        std::string response = sendCommandBlocking(drone.ip, "command");
        if (response == "ok") {
            drone.is_connected = true;
            connected_count++;
            std::cout << "Successfully connected to drone at " << drone.ip << std::endl;
        } else {
            std::cerr << "Failed to connect to drone at " << drone.ip << ": " << response << std::endl;
        }
    }
    
    std::cout << "\nInitialization complete: " << connected_count << " of " 
              << drones.size() << " drones are connected." << std::endl;
}

// Connect to VRPN server
bool DroneController::connectVrpn() {
    // Create socket for VRPN
    vrpn_socket = socket(AF_INET, SOCK_STREAM, 0);
    if (vrpn_socket < 0) {
        std::cerr << "Failed to create VRPN socket" << std::endl;
        return false;
    }

    struct sockaddr_in server_addr{};
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = inet_addr("192.168.1.100"); // OptiTrack server IP
    server_addr.sin_port = htons(3883);                       // VRPN port

    if (connect(vrpn_socket, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        std::cerr << "Failed to connect to VRPN server: " << strerror(errno) << std::endl;
        close(vrpn_socket);
        vrpn_socket = -1;
        return false;
    }

    // Set socket to non-blocking
    int flags = fcntl(vrpn_socket, F_GETFL, 0);
    fcntl(vrpn_socket, F_SETFL, flags | O_NONBLOCK);

    // Start a thread to receive VRPN data
    std::thread vrpn_thread([this]() {
        char buffer[4096];
        while (running.load()) {
            // Non-blocking read
            ssize_t received = recv(vrpn_socket, buffer, sizeof(buffer)-1, 0);
            if (received > 0) {
                buffer[received] = '\0';
                
                std::lock_guard<std::mutex> lock(tracker_mutex);
                // Parse tracker data from buffer
                std::string data(buffer);
                std::istringstream stream(data);
                std::string line;
                
                while (std::getline(stream, line)) {
                    std::istringstream linestream(line);
                    std::string name;
                    float x, y, z, qw, qx, qy, qz;
                    
                    std::getline(linestream, name, ',');
                    if (linestream >> x >> y >> z >> qw >> qx >> qy >> qz) {
                        VrpnData vrpn_data;
                        vrpn_data.position = {x, y, z};
                        vrpn_data.rotation = {qw, qx, qy, qz};
                        vrpn_data.timestamp = std::chrono::steady_clock::now();
                        tracker_data[name] = vrpn_data;
                        
                        // Update path for tracked drones
                        for (const auto& drone : drones) {
                            if (drone.tracker_id == name) {
                                // Store path point for visualization
                                if (drone_paths.find(name) == drone_paths.end()) {
                                    drone_paths[name] = std::deque<cv::Point2f>();
                                }
                                
                                // Add point to path
                                drone_paths[name].push_back(cv::Point2f(x, y));
                                
                                // Limit path length
                                if (drone_paths[name].size() > MAX_PATH_POINTS) {
                                    drone_paths[name].pop_front();
                                }
                                break;
                            }
                        }
                    }
                }
                
                has_vrpn_data.store(true);
            }
            
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    });
    
    // Detach thread so it runs independently
    vrpn_thread.detach();
    
    return true;
}

// Send command and wait for response (blocking)
std::string DroneController::sendCommandBlocking(const std::string& ip, const std::string& cmd, int timeout_ms) {
    if (drone_sockets.count(ip) == 0 || drone_sockets[ip] < 0) {
        return "Socket not initialized";
    }
    
    int sock = drone_sockets[ip];
    
    // Send command
    ssize_t sent = sendto(sock, cmd.c_str(), cmd.length(), 0,
                      (struct sockaddr *)&drone_addresses[ip], 
                      sizeof(drone_addresses[ip]));
    
    if (sent < 0) {
        return "Send failed: " + std::string(strerror(errno));
    }
    
    // Wait for response with poll
    struct pollfd pfd;
    pfd.fd = sock;
    pfd.events = POLLIN;
    
    if (poll(&pfd, 1, timeout_ms) > 0) {
        if (pfd.revents & POLLIN) {
            char buffer[1024];
            struct sockaddr_in from_addr;
            socklen_t from_len = sizeof(from_addr);
            
            ssize_t received = recvfrom(sock, buffer, sizeof(buffer)-1, 0,
                                  (struct sockaddr *)&from_addr, &from_len);
            
            if (received > 0) {
                buffer[received] = '\0';
                return std::string(buffer);
            }
        }
    }
    
    return "Timeout";
}

// Convert quaternion to yaw angle (around Z axis)
float DroneController::quaternionToYaw(const std::array<float, 4>& q) {
    // Extract yaw from quaternion (rotation around Z axis)
    // Formula: atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
    return std::atan2(2.0f * (q[0] * q[3] + q[1] * q[2]), 
                      1.0f - 2.0f * (q[2] * q[2] + q[3] * q[3]));
}

// Convert world coordinates to screen coordinates
cv::Point DroneController::worldToScreen(float x, float y) {
    cv::Point center(display.cols/2, display.rows/2);
    
    return cv::Point(
        center.x + static_cast<int>((x + OFFSET_X) * SCALE),
        center.y - static_cast<int>((y + OFFSET_Y) * SCALE)  // Flip Y for screen coordinates
    );
}

// Draw a grid and axes on the visualization
void DroneController::drawGrid() {
    cv::Point center(display.cols/2, display.rows/2);
    const int grid_size = 10;
    
    // Draw coordinate grid
    for (int i = -grid_size; i <= grid_size; i++) {
        // Vertical lines
        cv::Point p1 = worldToScreen(i, -grid_size);
        cv::Point p2 = worldToScreen(i, grid_size);
        cv::line(display, p1, p2, cv::Scalar(50, 50, 50), 1);
        
        // Horizontal lines
        p1 = worldToScreen(-grid_size, i);
        p2 = worldToScreen(grid_size, i);
        cv::line(display, p1, p2, cv::Scalar(50, 50, 50), 1);
    }

    // Draw axes
    cv::line(display, center, worldToScreen(grid_size, 0), cv::Scalar(0, 0, 255), 2);  // X axis
    cv::line(display, center, worldToScreen(0, grid_size), cv::Scalar(0, 255, 0), 2);  // Y axis
    
    cv::putText(display, "X", worldToScreen(grid_size + 0.5, 0), 
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255));
    cv::putText(display, "Y", worldToScreen(0, grid_size + 0.5), 
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0));
}

// Draw orientation arrow
void DroneController::drawOrientationArrow(const std::string& tracker_id, const cv::Point& pos, 
                                          const std::array<float, 4>& quaternion, const cv::Scalar& color) {
    // Calculate yaw from quaternion
    float yaw = quaternionToYaw(quaternion);
    
    // Arrow length
    float arrow_length = 0.5; // meters
    
    // Calculate arrow endpoint
    float end_x = pos.x + std::cos(yaw) * arrow_length * SCALE;
    float end_y = pos.y - std::sin(yaw) * arrow_length * SCALE; // Subtract for screen coords
    
    // Draw the arrow
    cv::arrowedLine(display, pos, cv::Point(end_x, end_y), color, 2);
    
    // Draw yaw angle text
    std::stringstream yaw_text;
    yaw_text << std::fixed << std::setprecision(1) << yaw * 180.0f / M_PI << "°";
    cv::putText(display, yaw_text.str(), cv::Point(pos.x + 15, pos.y - 15), 
               cv::FONT_HERSHEY_SIMPLEX, 0.5, color);
}

// Draw drone paths
void DroneController::drawDronePaths() {
    for (const auto& [tracker_id, path] : drone_paths) {
        // Find the drone with this tracker
        for (size_t i = 0; i < drones.size(); i++) {
            if (drones[i].tracker_id == tracker_id) {
                // Select color based on drone index
                cv::Scalar color;
                if (i == selected_drone_index) {
                    color = cv::Scalar(0, 255, 255); // Yellow for selected drone
                } else {
                    // Cycle through some colors
                    const cv::Scalar colors[] = {
                        cv::Scalar(255, 0, 0),   // Blue
                        cv::Scalar(0, 255, 0),   // Green
                        cv::Scalar(0, 0, 255),   // Red
                        cv::Scalar(255, 0, 255), // Magenta
                    };
                    color = colors[i % 4];
                }
                
                // Draw path as connected line segments
                if (path.size() >= 2) {
                    for (size_t j = 1; j < path.size(); j++) {
                        cv::Point p1 = worldToScreen(path[j-1].x, path[j-1].y);
                        cv::Point p2 = worldToScreen(path[j].x, path[j].y);
                        cv::line(display, p1, p2, color, 2);
                    }
                }
                break;
            }
        }
    }
}

// Update visualization display
void DroneController::updateVisualization() {
    // Clear display
    display = cv::Scalar(0, 0, 0);
    
    // Draw grid
    drawGrid();
    
    // Draw status text
    int text_y = 20;
    cv::putText(display, "Drone Controller", cv::Point(10, text_y), 
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255));
    text_y += 25;
    
    // Show selected drone status
    if (selected_drone_index >= 0 && selected_drone_index < static_cast<int>(drones.size())) {
        cv::putText(display, "Selected: " + drones[selected_drone_index].name + 
                   " (" + drones[selected_drone_index].ip + ")", 
                   cv::Point(10, text_y), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255));
    } else {
        cv::putText(display, "No drone selected", cv::Point(10, text_y), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(200, 200, 200));
    }
    text_y += 20;
    
    // Draw yaw transform if calculated
    if (yaw_transform != 0.0f) {
        std::stringstream transform_text;
        transform_text << "Yaw Transform: " << std::fixed << std::setprecision(2) 
                      << yaw_transform * 180.0f / M_PI << "°";
        cv::putText(display, transform_text.str(), cv::Point(10, text_y), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(200, 200, 0));
        text_y += 20;
    }
    
    // Draw drone paths
    drawDronePaths();
    
    // Draw trackers
    {
        std::lock_guard<std::mutex> lock(tracker_mutex);
        auto now = std::chrono::steady_clock::now();
        
        text_y = 70; // Reset text position for tracker list
        cv::putText(display, "Trackers:", cv::Point(10, text_y), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(200, 200, 200));
        text_y += 20;
        
        for (const auto& [name, data] : tracker_data) {
            // Calculate time since last update
            auto time_diff = std::chrono::duration_cast<std::chrono::milliseconds>(
                now - data.timestamp).count();
            
            // Skip if too old (3 seconds)
            if (time_diff > 3000) continue;
            
            // Find if this is a drone tracker
            bool is_drone = false;
            size_t drone_index = 0;
            for (size_t i = 0; i < drones.size(); i++) {
                if (drones[i].tracker_id == name) {
                    is_drone = true;
                    drone_index = i;
                    break;
                }
            }
            
            // Select color based on whether this is the selected drone
            cv::Scalar color;
            if (is_drone && drone_index == selected_drone_index) {
                color = cv::Scalar(0, 255, 255); // Yellow for selected drone
            } else if (is_drone) {
                // Cycle through some colors for drones
                const cv::Scalar colors[] = {
                    cv::Scalar(255, 0, 0),   // Blue
                    cv::Scalar(0, 255, 0),   // Green
                    cv::Scalar(0, 0, 255),   // Red
                    cv::Scalar(255, 0, 255), // Magenta
                };
                color = colors[drone_index % 4];
            } else {
                color = cv::Scalar(150, 150, 150); // Gray for non-drone markers
            }
            
            // Draw tracker position
            cv::Point pos = worldToScreen(data.position[0], data.position[1]);
            cv::circle(display, pos, 5, color, -1);
            
            // Draw orientation arrow for drones
            if (is_drone) {
                drawOrientationArrow(name, pos, data.rotation, color);
            }
            
            // Draw tracker name near the marker
            cv::putText(display, name, pos + cv::Point(10, -10), 
                       cv::FONT_HERSHEY_SIMPLEX, 0.4, color);
            
            // Draw z-coordinate
            std::stringstream z_text;
            z_text << "z: " << std::fixed << std::setprecision(2) << data.position[2];
            cv::putText(display, z_text.str(), pos + cv::Point(10, 10), 
                       cv::FONT_HERSHEY_SIMPLEX, 0.4, color);
            
            // Draw tracker info in the list
            std::string tracker_text = name + " [" + 
                std::to_string(data.position[0]).substr(0, 4) + ", " +
                std::to_string(data.position[1]).substr(0, 4) + ", " +
                std::to_string(data.position[2]).substr(0, 4) + "]";
            
            cv::putText(display, tracker_text, cv::Point(10, text_y), 
                       cv::FONT_HERSHEY_SIMPLEX, 0.4, color);
            text_y += 20;
        }
    }
    
    // Show the visualization
    cv::imshow(window_name, display);
    cv::waitKey(1);
}

// Launch VRPN visualization in a separate process
pid_t DroneController::launchVrpnViz() {
    pid_t pid = fork();
    
    if (pid == 0) {
        // Child process - redirect stdout and stderr to /dev/null to avoid console spam
        int null_fd = open("/dev/null", O_WRONLY);
        dup2(null_fd, STDOUT_FILENO);
        dup2(null_fd, STDERR_FILENO);
        close(null_fd);
        
        const char* vrpn_viz_path = "/home/finn/squawkblock/squawkblock/vrpn/build/vrpn_viz";
        execl(vrpn_viz_path, vrpn_viz_path, NULL);
        
        // If exec fails
        exit(1);
    }
    
    // Parent process - return child PID
    return pid;
}

// Calculate transform from OptiTrack frame to command frame
void DroneController::calculateYawTransform(const std::string& tracker_id, float forward_direction) {
    std::lock_guard<std::mutex> lock(tracker_mutex);
    
    if (tracker_data.find(tracker_id) == tracker_data.end()) {
        std::cerr << "No tracker data available for " << tracker_id << std::endl;
        return;
    }
    
    // Get orientation from tracker
    float yaw = quaternionToYaw(tracker_data[tracker_id].rotation);
    
    // Calculate the transform
    // forward_direction is the yaw angle in the command frame that corresponds to "forward"
    // We need to find the difference between the OptiTrack yaw and this forward direction
    yaw_transform = forward_direction - yaw;
    
    // Normalize to [-π, π]
    while (yaw_transform > M_PI) yaw_transform -= 2 * M_PI;
    while (yaw_transform < -M_PI) yaw_transform += 2 * M_PI;
    
    std::cout << "Calculated yaw transform: " << yaw_transform * 180.0f / M_PI 
             << " degrees (" << yaw_transform << " radians)" << std::endl;
    
    // Save to file
    std::ofstream file("yaw_transform.txt");
    if (file.is_open()) {
        file << yaw_transform << std::endl;
        file.close();
        std::cout << "Saved yaw transform to yaw_transform.txt" << std::endl;
    } else {
        std::cerr << "Failed to save yaw transform to file" << std::endl;
    }
}

// Show available drones
void DroneController::showAvailableDrones() {
    std::cout << "\nAvailable Drones:" << std::endl;
    for (size_t i = 0; i < drones.size(); i++) {
        std::cout << i + 1 << ". " << drones[i].name << " (IP: " << drones[i].ip 
                 << ", Tracker: " << drones[i].tracker_id << ") - " 
                 << (drones[i].is_connected ? "Connected" : "Disconnected") << std::endl;
    }
    std::cout << "0. Cancel" << std::endl;
}

// Select a drone from the available drones
bool DroneController::selectDrone() {
    showAvailableDrones();
    std::cout << "Enter the number of the drone to control: ";
    
    int selection;
    std::cin >> selection;
    
    if (selection < 1 || selection > static_cast<int>(drones.size())) {
        std::cout << "Selection canceled or invalid." << std::endl;
        return false;
    }
    
    selected_drone_index = selection - 1;
    std::cout << "Selected drone: " << drones[selected_drone_index].name 
              << " (" << drones[selected_drone_index].ip << ")" << std::endl;
    
    if (!drones[selected_drone_index].is_connected) {
        std::cout << "Warning: This drone is not connected!" << std::endl;
        return false;
    }
    
    return true;
}

// Flight sequence: takeoff, move up, move forward
void DroneController::executeFlightSequence() {
    if (selected_drone_index < 0 || selected_drone_index >= static_cast<int>(drones.size())) {
        std::cerr << "No drone selected!" << std::endl;
        return;
    }
    
    auto& drone = drones[selected_drone_index];
    if (!drone.is_connected) {
        std::cerr << "Selected drone is not connected!" << std::endl;
        return;
    }
    
    std::cout << "Starting flight sequence for " << drone.name << " (" << drone.ip << ")" << std::endl;
    
    // Clear path data for this drone
    if (drone_paths.find(drone.tracker_id) != drone_paths.end()) {
        drone_paths[drone.tracker_id].clear();
    }
    
    // Make sure the visualization is active
    visualization_active = true;
    
    // Send command mode first
    std::cout << "Sending command mode..." << std::endl;
    std::string cmd_response = sendCommandBlocking(drone.ip, "command");
    if (cmd_response != "ok") {
        std::cerr << "Command mode failed: " << cmd_response << std::endl;
        return;
    }
    
    // Wait a moment
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    // Initiate takeoff
    std::cout << "Taking off..." << std::endl;
    std::string takeoff_response = sendCommandBlocking(drone.ip, "takeoff");
    if (takeoff_response != "ok") {
        std::cerr << "Takeoff failed: " << takeoff_response << std::endl;
        return;
    }
    
    // Wait for takeoff to complete
    std::cout << "Waiting for takeoff to complete..." << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(TAKEOFF_WAIT_MS));
    
    // Move upward for 2 seconds
    std::cout << "Moving upward..." << std::endl;
    sendCommandBlocking(drone.ip, "rc 0 0 " + std::to_string(UP_SPEED) + " 0");
    std::this_thread::sleep_for(std::chrono::milliseconds(MOVE_UP_DURATION_MS));
    
    // Stop vertical movement
    sendCommandBlocking(drone.ip, "rc 0 0 0 0");
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    // Record orientation before moving forward
    float initial_yaw = 0.0f;
    {
        std::lock_guard<std::mutex> lock(tracker_mutex);
        if (tracker_data.find(drone.tracker_id) != tracker_data.end()) {
            initial_yaw = quaternionToYaw(tracker_data[drone.tracker_id].rotation);
        }
    }
    
    // Move forward for 5 seconds
    std::cout << "Moving forward..." << std::endl;
    sendCommandBlocking(drone.ip, "rc 0 " + std::to_string(FORWARD_SPEED) + " 0 0");
    std::this_thread::sleep_for(std::chrono::milliseconds(MOVE_FORWARD_DURATION_MS));
    
    // Stop all movement
    sendCommandBlocking(drone.ip, "rc 0 0 0 0");
    
    // Calculate yaw transform
    calculateYawTransform(drone.tracker_id, initial_yaw);
    
    // Land
    std::cout << "Landing..." << std::endl;
    sendCommandBlocking(drone.ip, "land");
    
    std::cout << "Flight sequence completed." << std::endl;
}

// Reboot the selected drone
void DroneController::rebootDrone() {
    if (selected_drone_index < 0 || selected_drone_index >= static_cast<int>(drones.size())) {
        std::cerr << "No drone selected!" << std::endl;
        return;
    }
    
    auto& drone = drones[selected_drone_index];
    
    std::cout << "Rebooting drone " << drone.name << " (" << drone.ip << ")..." << std::endl;
    
    // Create a direct reboot socket
    int reboot_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (reboot_sock < 0) {
        std::cerr << "Failed to create socket for reboot" << std::endl;
        return;
    }
    
    // Set up address
    struct sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = inet_addr(drone.ip.c_str());
    addr.sin_port = htons(8889);
    
    // Send reboot command
    std::string cmd = "reboot";
    ssize_t sent = sendto(reboot_sock, cmd.c_str(), cmd.length(), 0, 
                    (struct sockaddr*)&addr, sizeof(addr));
    
    close(reboot_sock);
    
    if (sent > 0) {
        std::cout << "Reboot command sent to " << drone.ip << std::endl;
        std::cout << "Waiting for drone to reboot (8 seconds)..." << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(8));
        
        // Reinitialize the socket
        if (initSocket(drone)) {
            // Test connection after reboot
            std::string response = sendCommandBlocking(drone.ip, "command");
            if (response == "ok") {
                drone.is_connected = true;
                std::cout << "Drone reconnected successfully after reboot." << std::endl;
            } else {
                drone.is_connected = false;
                std::cerr << "Failed to reconnect to drone after reboot: " << response << std::endl;
            }
        } else {
            std::cerr << "Failed to reinitialize socket after reboot" << std::endl;
            drone.is_connected = false;
        }
    } else {
        std::cerr << "Failed to send reboot command: " << strerror(errno) << std::endl;
    }
}

// Main menu options
void DroneController::showMenu() {
    while (running.load()) {
        if (visualization_active) {
            updateVisualization();
        }
        
        std::cout << "\n==============================" << std::endl;
        std::cout << "Drone Controller Main Menu" << std::endl;
        std::cout << "==============================" << std::endl;
        
        // Show selected drone status
        if (selected_drone_index >= 0 && selected_drone_index < static_cast<int>(drones.size())) {
            std::cout << "Selected drone: " << drones[selected_drone_index].name 
                     << " (" << drones[selected_drone_index].ip << ")" << std::endl;
        } else {
            std::cout << "No drone selected" << std::endl;
        }
        
        std::cout << "\n1. Select drone" << std::endl;
        std::cout << "2. Show OptiTrack visualization" << std::endl;
        std::cout << "3. Execute flight sequence" << std::endl;
        std::cout << "4. Reboot drone" << std::endl;
        std::cout << "0. Exit" << std::endl;
        
        std::cout << "\nEnter choice: ";
        
        int choice;
        std::cin >> choice;
        
        switch (choice) {
            case 1:
                selectDrone();
                break;
            case 2:
                if (!visualization_active) {
                    std::cout << "Starting visualization..." << std::endl;
                    visualization_active = true;
                } else {
                    std::cout << "Visualization already active" << std::endl;
                }
                break;
            case 3:
                executeFlightSequence();
                break;
            case 4:
                rebootDrone();
                break;
            case 0:
                std::cout << "Exiting..." << std::endl;
                running = false;
                return;
            default:
                std::cout << "Invalid choice. Please try again." << std::endl;
                break;
        }
    }
}

// Main run function
void DroneController::run() {
    showMenu();
}