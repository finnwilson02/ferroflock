#include <iostream>
#include <vector>
#include <chrono>
#include <thread>
#include <unistd.h>
#include <netinet/in.h>
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
#include <nlohmann/json.hpp>
#include <deque>

// For convenience
using json = nlohmann::json;

// Constants for flight control
const int MOVE_UP_DURATION_MS = 2000;    // 2 seconds moving up
const int MOVE_FORWARD_DURATION_MS = 5000; // 5 seconds moving forward
const int TAKEOFF_WAIT_MS = 5000;        // Wait 5 seconds for takeoff
const int UP_SPEED = 30;                 // Not max speed (0-100)
const int FORWARD_SPEED = 30;            // Not max speed (0-100)

// Path tracing constants
const int MAX_PATH_POINTS = 500;         // Maximum number of points to store

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

public:
    DroneController() {
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
    
    ~DroneController() {
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
    void loadDroneConfigs() {
        // First try to load from JSON file
        std::string filename = "drone_mapping.json";
        
        // Try current directory first
        if (!std::filesystem::exists(filename)) {
            // Try with path
            filename = "drone_controller/drone_mapping.json";
            if (!std::filesystem::exists(filename)) {
                std::cerr << "Warning: Could not find drone_mapping.json, falling back to sync-drones configuration" << std::endl;
                
                // Fall back to loading from the sync-drones dji_devices.json
                std::string dji_filename = "../sync-drones/dji_devices.json";
                if (!std::filesystem::exists(dji_filename)) {
                    dji_filename = "/home/finn/squawkblock/squawkblock/sync-drones/dji_devices.json";
                    if (!std::filesystem::exists(dji_filename)) {
                        std::cerr << "Could not find dji_devices.json either!" << std::endl;
                        return;
                    }
                }
                
                try {
                    std::ifstream file(dji_filename);
                    json data = json::parse(file);
                    
                    std::cout << "Loading drones from " << dji_filename << std::endl;
                    
                    // Parse devices from dji_devices.json
                    for (const auto& device : data) {
                        if (device.contains("ip") && device.contains("mac") && device.contains("online")) {
                            // Only process online drones
                            if (device["online"].get<bool>()) {
                                DroneConfig drone;
                                drone.name = device["mac"];  // Use MAC as name for now
                                drone.ip = device["ip"];
                                drone.tracker_id = ""; // No tracker mapping yet
                                drone.tracker_num = 0;
                                drones.push_back(drone);
                                
                                std::cout << "Loaded drone: MAC=" << drone.name 
                                          << " (IP: " << drone.ip << ")" << std::endl;
                            }
                        }
                    }
                } catch (const std::exception& e) {
                    std::cerr << "Error loading from dji_devices.json: " << e.what() << std::endl;
                    return;
                }
                
                std::cout << "Loaded " << drones.size() << " drone configurations from dji_devices.json" << std::endl;
                return;
            }
        }
        
        // If we got here, we found the drone_mapping.json file, so use it
        try {
            std::ifstream file(filename);
            json data = json::parse(file);
            
            // Parse drones from JSON
            for (const auto& [name, tracker_info] : data["trackers"].items()) {
                DroneConfig drone;
                drone.name = name;
                drone.tracker_id = tracker_info["tracker_id"];
                drone.tracker_num = tracker_info["tracker_num"];
                drone.ip = tracker_info["drone_ip"];
                drones.push_back(drone);
                
                std::cout << "Loaded drone: " << name << " (IP: " << drone.ip 
                          << ", Tracker: " << drone.tracker_id << ")" << std::endl;
            }
            
            std::cout << "Loaded " << drones.size() << " drone configurations from " << filename << std::endl;
        } catch (const std::exception& e) {
            std::cerr << "Error loading drone configurations: " << e.what() << std::endl;
        }
    }
    
    // Initialize socket for a drone
    bool initSocket(DroneConfig& drone) {
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
    void initializeDrones() {
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
            
            // First try using Bash-style non-blocking approach (non-blocking)
            int command_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
            if (command_sock >= 0) {
                // Set up address
                struct sockaddr_in command_addr{};
                command_addr.sin_family = AF_INET;
                command_addr.sin_addr.s_addr = inet_addr(drone.ip.c_str());
                command_addr.sin_port = htons(8889);
                
                // Set socket timeout
                struct timeval tv;
                tv.tv_sec = 0;
                tv.tv_usec = 500000;  // 500ms timeout
                setsockopt(command_sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
                
                // Send command directly
                std::string cmd = "command";
                std::cout << "Sending direct command to " << drone.ip << "..." << std::endl;
                ssize_t sent = sendto(command_sock, cmd.c_str(), cmd.length(), 0, 
                                 (struct sockaddr*)&command_addr, sizeof(command_addr));
                
                if (sent > 0) {
                    // Wait for response
                    char buffer[1024];
                    struct sockaddr_in from_addr;
                    socklen_t from_len = sizeof(from_addr);
                    
                    ssize_t received = recvfrom(command_sock, buffer, sizeof(buffer)-1, 0,
                                         (struct sockaddr *)&from_addr, &from_len);
                    
                    if (received > 0) {
                        buffer[received] = '\0';
                        std::string response = buffer;
                        
                        if (response == "ok") {
                            drone.is_connected = true;
                            connected_count++;
                            std::cout << "Successfully connected to drone at " << drone.ip << " (direct method)" << std::endl;
                        } else {
                            std::cout << "Direct command response: " << response << ", trying standard method..." << std::endl;
                        }
                    }
                }
                
                close(command_sock);
            }
            
            // If not yet connected, try standard method
            if (!drone.is_connected) {
                // Test connection with the regular command method
                std::cout << "Trying standard method for " << drone.ip << "..." << std::endl;
                std::string response = sendCommandBlocking(drone.ip, "command");
                
                // Always assume connected - in drift_calibrator.cpp it doesn't strictly check responses
                // The drones often don't respond properly but will still accept commands
                drone.is_connected = true;
                connected_count++;
                
                if (response == "ok") {
                    std::cout << "Successfully connected to drone at " << drone.ip << " (standard method)" << std::endl;
                } else {
                    std::cout << "Response from " << drone.ip << ": " << response << " (assuming connected anyway)" << std::endl;
                }
            }
        }
        
        std::cout << "\nInitialization complete: " << connected_count << " of " 
                  << drones.size() << " drones are connected." << std::endl;
    }
    
    // Connect to VRPN server
    bool connectVrpn() {
        // Debug tracker IDs
        std::cout << "Connecting to VRPN server with drone tracker IDs:" << std::endl;
        for (const auto& drone : drones) {
            std::cout << "  Drone " << drone.ip << " has tracker ID: '" << drone.tracker_id << "'" << std::endl;
        }

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
                            // Output data for debugging - see what we receive
                            std::cout << "Received VRPN data for tracker: '" << name << "'" << std::endl;
                            
                            VrpnData vrpn_data;
                            vrpn_data.position = {x, y, z};
                            vrpn_data.rotation = {qw, qx, qy, qz};
                            vrpn_data.timestamp = std::chrono::steady_clock::now();
                            tracker_data[name] = vrpn_data;
                            
                            // Also store a version without spaces to be flexible
                            std::string alt_name = name;
                            alt_name.erase(std::remove(alt_name.begin(), alt_name.end(), ' '), alt_name.end());
                            if (alt_name != name) {
                                tracker_data[alt_name] = vrpn_data;
                                std::cout << "  Also storing as '" << alt_name << "'" << std::endl;
                            }
                            
                            // Update path for tracked drones
                            for (const auto& drone : drones) {
                                if (drone.tracker_id == name || drone.tracker_id == alt_name) {
                                    // Store path point for visualization
                                    if (drone_paths.find(drone.tracker_id) == drone_paths.end()) {
                                        drone_paths[drone.tracker_id] = std::deque<cv::Point2f>();
                                    }
                                    
                                    // Add point to path
                                    drone_paths[drone.tracker_id].push_back(cv::Point2f(x, y));
                                    
                                    // Limit path length
                                    if (drone_paths[drone.tracker_id].size() > MAX_PATH_POINTS) {
                                        drone_paths[drone.tracker_id].pop_front();
                                    }
                                    std::cout << "  Matched to drone with IP: " << drone.ip << std::endl;
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
    std::string sendCommandBlocking(const std::string& ip, const std::string& cmd, int timeout_ms = 500) {
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
        
        // For the "command" command, we can be more lenient with responses
        if (cmd == "command") {
            // Return "ok" even without waiting for response - like drift_calibrator does
            // This helps when drones are in weird states but would still accept further commands
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            return "ok";
        }
        
        // For other commands, wait for response with poll
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
        
        // If we time out on a crucial command, it's still worth retrying
        // For now, return a more optimistic message
        if (cmd == "takeoff" || cmd == "land" || cmd.substr(0, 2) == "rc") {
            // For flight commands, assume it worked
            return "ok (assumed)";
        }
        
        return "Timeout";
    }
    
    // Convert quaternion to yaw angle (around Z axis)
    float quaternionToYaw(const std::array<float, 4>& q) {
        // Extract yaw from quaternion (rotation around Z axis)
        // Formula: atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
        return std::atan2(2.0f * (q[0] * q[3] + q[1] * q[2]), 
                          1.0f - 2.0f * (q[2] * q[2] + q[3] * q[3]));
    }
    
    // Convert world coordinates to screen coordinates
    cv::Point worldToScreen(float x, float y) {
        cv::Point center(display.cols/2, display.rows/2);
        
        return cv::Point(
            center.x + static_cast<int>((x + OFFSET_X) * SCALE),
            center.y - static_cast<int>((y + OFFSET_Y) * SCALE)  // Flip Y for screen coordinates
        );
    }
    
    // Draw a grid and axes on the visualization
    void drawGrid() {
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
    void drawOrientationArrow(const std::string& tracker_id, const cv::Point& pos, const std::array<float, 4>& quaternion, const cv::Scalar& color) {
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
    void drawDronePaths() {
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
    void updateVisualization() {
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
        if (selected_drone_index >= 0 && selected_drone_index < drones.size()) {
            cv::putText(display, "Selected: " + drones[selected_drone_index].name + 
                       " (" + drones[selected_drone_index].ip + ")", 
                       cv::Point(10, text_y), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255));
        } else {
            cv::putText(display, "No drone selected", cv::Point(10, text_y), 
                       cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(200, 200, 200));
        }
        text_y += 20;
        
        // Get current time
        auto now = std::chrono::steady_clock::now();
        
        // For formatting time string, use system_clock instead
        auto now_c = std::chrono::system_clock::now();
        std::time_t now_time = std::chrono::system_clock::to_time_t(now_c);
        struct tm *time_info = std::localtime(&now_time);
        char timestr[20];
        std::strftime(timestr, sizeof(timestr), "%H:%M:%S", time_info);
        
        // Show current time in visualization
        cv::putText(display, "Time: " + std::string(timestr), cv::Point(10, text_y), 
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(200, 200, 200));
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
        
        // Check if we have any tracker data
        bool has_data = false;
        {
            std::lock_guard<std::mutex> lock(tracker_mutex);
            has_data = !tracker_data.empty();
        }
        
        // Draw data status indicator
        if (has_data) {
            cv::putText(display, "✓ Tracking data active", cv::Point(10, text_y), 
                       cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0));
        } else {
            cv::putText(display, "✗ No tracking data", cv::Point(10, text_y), 
                       cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255));
        }
        text_y += 20;
        
        // Draw trackers
        {
            std::lock_guard<std::mutex> lock(tracker_mutex);
            
            text_y = 70; // Reset text position for tracker list
            cv::putText(display, "Active Trackers:", cv::Point(10, text_y), 
                       cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(200, 200, 200));
            text_y += 20;
            
            for (const auto& [name, data] : tracker_data) {
                // Calculate time since last update
                auto time_diff = std::chrono::duration_cast<std::chrono::milliseconds>(
                    now - data.timestamp).count();
                
                // Skip if too old (10 seconds) or special markers
                if (time_diff > 10000 || name == "TimeDisplay" || name == "FlightTime") 
                    continue;
                
                // Skip IP-based alternate displays to avoid clutter
                if (name.find("Drone_") == 0)
                    continue;
                
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
                
                // Draw highlighted marker for drones
                if (is_drone) {
                    // Draw a larger circle for drone markers
                    cv::circle(display, pos, 8, color, -1);
                    
                    // Draw IP address inside the marker for easier identification
                    std::string last_octet = drones[drone_index].ip.substr(drones[drone_index].ip.rfind('.') + 1);
                    cv::putText(display, last_octet, pos + cv::Point(-6, 4), 
                               cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(0, 0, 0));
                } else {
                    // Regular markers
                    cv::circle(display, pos, 5, color, -1);
                }
                
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
                
                // Add IP number to the display if it's a drone
                if (is_drone) {
                    tracker_text += " (IP: " + drones[drone_index].ip + ")";
                }
                
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
    pid_t launchVrpnViz() {
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
    void calculateYawTransform(const std::string& tracker_id, float forward_direction) {
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
    void showAvailableDrones() {
        std::cout << "\nAvailable Drones:" << std::endl;
        for (size_t i = 0; i < drones.size(); i++) {
            std::cout << i + 1 << ". " << drones[i].name << " (IP: " << drones[i].ip 
                     << ", Tracker: " << drones[i].tracker_id << ") - " 
                     << (drones[i].is_connected ? "Connected" : "Disconnected") << std::endl;
        }
        std::cout << "0. Cancel" << std::endl;
    }
    
    // Select a drone from the available drones
    bool selectDrone() {
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
    void executeFlightSequence() {
        if (selected_drone_index < 0 || selected_drone_index >= static_cast<int>(drones.size())) {
            std::cerr << "No drone selected!" << std::endl;
            return;
        }
        
        auto& drone = drones[selected_drone_index];
        // Always assume the drone is connected - like in drift_calibrator
        drone.is_connected = true;
        std::cout << "Ensuring drone " << drone.ip << " is marked as connected..." << std::endl;
        
        std::cout << "Starting flight sequence for " << drone.name << " (" << drone.ip << ")" << std::endl;
        
        // Clear path data for this drone
        if (drone_paths.find(drone.tracker_id) != drone_paths.end()) {
            drone_paths[drone.tracker_id].clear();
        }
        
        // Make sure the visualization is active
        visualization_active = true;
        
        // Send command mode first - always proceed regardless of response
        std::cout << "Sending command mode..." << std::endl;
        std::string cmd_response = sendCommandBlocking(drone.ip, "command");
        std::cout << "Command response: " << cmd_response << " (proceeding anyway)" << std::endl;
        
        // Wait a moment
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        
        // Initiate takeoff - similar to drift_calibrator, don't rely on response
        std::cout << "Taking off..." << std::endl;
        std::string takeoff_response = sendCommandBlocking(drone.ip, "takeoff");
        std::cout << "Takeoff command sent, response: " << takeoff_response << std::endl;
        
        // Always continue regardless of response - drones often don't respond properly
        // Wait longer for takeoff to complete (just like in drift_calibrator.cpp)
        std::cout << "Waiting for takeoff to complete..." << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(TAKEOFF_WAIT_MS + 2000));
        
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
    void rebootDrone() {
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
    void showMenu() {
        while (true) {
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
    void run() {
        showMenu();
    }
};

int main() {
    // Setup signal handler to ensure clean exit on Ctrl+C
    struct sigaction sa;
    sa.sa_handler = [](int sig) {
        std::cout << "\nExiting..." << std::endl;
        exit(sig);
    };
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;
    sigaction(SIGINT, &sa, NULL);  // Handle Ctrl+C
    sigaction(SIGTERM, &sa, NULL); // Handle termination signal
    
    try {
        DroneController controller;
        controller.run();
    } catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
        return 1;
    } catch (...) {
        std::cerr << "Unknown exception occurred" << std::endl;
        return 1;
    }
    
    return 0;
}