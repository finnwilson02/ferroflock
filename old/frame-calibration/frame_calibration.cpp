#include <vrpn_Tracker.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <chrono>
#include <vector>
#include <cmath>
#include <algorithm>
#include <fstream>
#include <optional>
#include <unistd.h>
#include <fcntl.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <cstring>
#include <csignal>
#include "include/FileHandler.hpp"

// Simple JSON implementation instead of nlohmann/json
class JsonValue {
public:
    std::string str_value;
    double num_value = 0.0;
    bool is_string = false;
    
    JsonValue() = default;
    JsonValue(const std::string& s) : str_value(s), is_string(true) {}
    JsonValue(double n) : num_value(n), is_string(false) {}
};

class JsonArray {
private:
    std::vector<std::map<std::string, JsonValue>> items;
    
public:
    void push_back(const std::map<std::string, JsonValue>& item) {
        items.push_back(item);
    }
    
    std::string dump(int indent = 0) const {
        std::string result = "[\n";
        for (size_t i = 0; i < items.size(); ++i) {
            result += std::string(indent + 2, ' ') + "{\n";
            
            const auto& item = items[i];
            size_t count = 0;
            for (const auto& [key, value] : item) {
                result += std::string(indent + 4, ' ') + "\"" + key + "\": ";
                if (value.is_string) {
                    result += "\"" + value.str_value + "\"";
                } else {
                    result += std::to_string(value.num_value);
                }
                
                if (++count < item.size()) {
                    result += ",";
                }
                result += "\n";
            }
            
            result += std::string(indent + 2, ' ') + "}";
            if (i < items.size() - 1) {
                result += ",";
            }
            result += "\n";
        }
        result += std::string(indent, ' ') + "]";
        return result;
    }
};

// Forward declarations
cv::Point worldToScreen(double x, double y, const cv::Point& center);
double quaternionToYaw(double qw, double qx, double qy, double qz);
double correctYawFlip(double measured_yaw, double previous_yaw);
void saveYawLogToFile(const std::string& tracker_id);
void sendDroneCommand(const std::string& ip, const std::string& command, bool skip_reboot);
void printMenu();
void rebootAllDrones();

// Structure to hold position and orientation data
struct TrackerData {
    double x{0}, y{0}, z{0};
    double qx{0}, qy{0}, qz{0}, qw{1.0}; // Quaternion for orientation
    cv::Scalar color;
    bool updated{false};
    std::chrono::system_clock::time_point last_update;
    std::vector<cv::Point> path;  // Store path history
    static const size_t MAX_PATH_LENGTH = 2500;  // Maximum number of points to store
    
    // Calibration-specific data
    std::vector<double> yaw_values; // Store yaw values during calibration
    std::vector<std::chrono::system_clock::time_point> yaw_timestamps;
    
    // Yaw correction data
    double last_raw_yaw{0.0};       // Last uncorrected yaw from quaternion
    double last_corrected_yaw{0.0}; // Last corrected yaw after flip detection
    bool has_prior_yaw{false};      // Flag to indicate if we have a prior yaw measurement
    
    // Logging data for yaw correction analysis
    std::vector<double> raw_yaw_log;
    std::vector<double> corrected_yaw_log;
    std::vector<std::chrono::system_clock::time_point> yaw_log_timestamps;
    std::string log_filename;       // Filename for storing yaw correction data
};

// DroneData structure to combine tracker and drone info
struct DroneData {
    std::string name;        // Human-readable name (e.g., "Bird 3")
    std::string tracker_id;  // OptiTrack tracker ID (e.g., "Tracker4")
    std::string ip;          // IP address of the drone (e.g., "192.168.1.108")
    double yaw_offset{0.0};  // Calibrated yaw offset
};

// Global data with mutex protection
std::map<std::string, TrackerData> g_trackers;
std::mutex g_mutex;
int selected_tracker_idx = -1; // -1 means no tracker selected
std::vector<DroneData> g_drones;  // Stores the drone-to-tracker mapping
bool calibration_mode = false;    // Flag for when we're in calibration mode
std::chrono::system_clock::time_point calibration_start_time;
std::chrono::system_clock::time_point takeoff_time;
double calibration_drone_yaw = 0.0;  // The drone's command frame forward direction
bool drone_flying = false;

// Visualization settings
const double SCALE = 50.0;  // Scale factor for visualization
const double OFFSET_X = 0.0;  // Offset to center the visualization
const double OFFSET_Y = 0.0;

// Path to calibration file
const std::string CALIBRATION_FILE = "calibration/frame_calibration.json";

// VRPN callback function
void VRPN_CALLBACK handle_tracker(void* userData, const vrpn_TRACKERCB t) {
    std::string* name = static_cast<std::string*>(userData);
    std::lock_guard<std::mutex> lock(g_mutex);
    
    // Get the current timestamp
    auto now = std::chrono::system_clock::now();
    
    bool was_updated = g_trackers[*name].updated;
    
    // Update position data
    g_trackers[*name].x = t.pos[0];
    g_trackers[*name].y = t.pos[1];
    g_trackers[*name].z = t.pos[2];
    g_trackers[*name].qx = t.quat[0];
    g_trackers[*name].qy = t.quat[1];
    g_trackers[*name].qz = t.quat[2];
    g_trackers[*name].qw = t.quat[3];
    g_trackers[*name].updated = true;
    g_trackers[*name].last_update = now;
    
    // Print debug info if this is first time it becomes visible
    if (!was_updated && calibration_mode) {
        std::cout << "DEBUG: Tracker '" << *name << "' became visible! Position: (" 
                  << t.pos[0] << ", " << t.pos[1] << ", " << t.pos[2] << ")" << std::endl;
    }
    
    // Add current position to path history
    cv::Point center(500, 400); // Must match the center in main loop (display.cols/2, display.rows/2)
    cv::Point pos = worldToScreen(t.pos[0], t.pos[1], center);
    g_trackers[*name].path.push_back(pos);
    
    // Keep path length limited
    if (g_trackers[*name].path.size() > TrackerData::MAX_PATH_LENGTH) {
        g_trackers[*name].path.erase(g_trackers[*name].path.begin());
    }
    
    // Calculate the raw yaw from quaternion
    double raw_yaw = quaternionToYaw(t.quat[3], t.quat[0], t.quat[1], t.quat[2]);
    
    // Apply yaw flip correction if we have a prior measurement
    double corrected_yaw = raw_yaw;
    if (g_trackers[*name].has_prior_yaw) {
        corrected_yaw = correctYawFlip(raw_yaw, g_trackers[*name].last_corrected_yaw);
    }
    
    // Save the raw and corrected yaw values for next time
    g_trackers[*name].last_raw_yaw = raw_yaw;
    g_trackers[*name].last_corrected_yaw = corrected_yaw;
    g_trackers[*name].has_prior_yaw = true;
    
    // Log the yaw data for analysis
    g_trackers[*name].raw_yaw_log.push_back(raw_yaw);
    g_trackers[*name].corrected_yaw_log.push_back(corrected_yaw);
    g_trackers[*name].yaw_log_timestamps.push_back(now);
    
    // If in calibration mode, record corrected yaw values for the specific drone
    if (calibration_mode && drone_flying) {
        // Only record during the "forward flight" portion of calibration
        // After upward movement and during forward motion
        auto elapsed_since_takeoff = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - takeoff_time).count();
            
        // During middle 4 seconds of forward flight (after upward movement of 3s + 1s stabilize)
        // This is approximately 8-12s after takeoff
        if (elapsed_since_takeoff >= 8000 && elapsed_since_takeoff <= 12000) {
            // We use the corrected yaw for calibration to avoid flip issues
            g_trackers[*name].yaw_values.push_back(corrected_yaw);
            g_trackers[*name].yaw_timestamps.push_back(now);
        }
    }
}

// Convert world coordinates to screen coordinates
cv::Point worldToScreen(double x, double y, const cv::Point& center) {
    return cv::Point(
        center.x + static_cast<int>((x + OFFSET_X) * SCALE),
        center.y - static_cast<int>((y + OFFSET_Y) * SCALE)  // Flip Y for screen coordinates
    );
}

// Extract yaw angle from quaternion
double quaternionToYaw(double qw, double qx, double qy, double qz) {
    // Extract yaw from quaternion - this shows the heading direction in the XY plane
    double siny_cosp = 2.0 * (qw * qz + qx * qy);
    double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
    return std::atan2(siny_cosp, cosy_cosp);
}

// Correct yaw flip by minimizing the angular difference
double correctYawFlip(double measured_yaw, double previous_yaw) {
    // Normalize angles to [-π, π]
    double normalized_current = measured_yaw;
    while (normalized_current > M_PI) normalized_current -= 2.0 * M_PI;
    while (normalized_current < -M_PI) normalized_current += 2.0 * M_PI;
    
    double normalized_prev = previous_yaw;
    while (normalized_prev > M_PI) normalized_prev -= 2.0 * M_PI;
    while (normalized_prev < -M_PI) normalized_prev += 2.0 * M_PI;
    
    // Calculate angular difference between current and previous
    double diff = normalized_current - normalized_prev;
    while (diff > M_PI) diff -= 2.0 * M_PI;
    while (diff < -M_PI) diff += 2.0 * M_PI;
    
    // Calculate difference if we applied a π flip
    double flipped_current = normalized_current + M_PI;
    if (flipped_current > M_PI) flipped_current -= 2.0 * M_PI;
    
    double flipped_diff = flipped_current - normalized_prev;
    while (flipped_diff > M_PI) flipped_diff -= 2.0 * M_PI;
    while (flipped_diff < -M_PI) flipped_diff += 2.0 * M_PI;
    
    // Choose the yaw value that minimizes the angular difference
    // Use absolute values for comparison
    if (std::abs(diff) < std::abs(flipped_diff)) {
        return normalized_current;
    } else {
        return flipped_current;
    }
}

// Log yaw data for analysis - this function is not needed since we log in the callback

// Save yaw log data to text file with enhanced position and command data
void saveYawLogToFile(const std::string& tracker_id) {
    // First check if tracker exists and has data
    {
        std::lock_guard<std::mutex> lock(g_mutex);
        if (g_trackers.find(tracker_id) == g_trackers.end()) {
            std::cerr << "ERROR: Cannot save log - tracker '" << tracker_id << "' not found!" << std::endl;
            return;
        }
        
        if (g_trackers[tracker_id].raw_yaw_log.empty()) {
            std::cerr << "ERROR: No yaw data collected for " << tracker_id 
                    << ". Log size: " << g_trackers[tracker_id].raw_yaw_log.size() << std::endl;
            
            // Print debug info about the tracker's state
            std::cout << "DEBUG: Tracker '" << tracker_id << "' data:" << std::endl;
            std::cout << "  - Updated: " << (g_trackers[tracker_id].updated ? "YES" : "NO") << std::endl;
            std::cout << "  - Has prior yaw: " << (g_trackers[tracker_id].has_prior_yaw ? "YES" : "NO") << std::endl;
            std::cout << "  - Position: (" << g_trackers[tracker_id].x << ", " 
                    << g_trackers[tracker_id].y << ", " 
                    << g_trackers[tracker_id].z << ")" << std::endl;
            
            return;
        }
        
        std::cout << "INFO: Found " << g_trackers[tracker_id].raw_yaw_log.size() 
                << " data points to save for tracker '" << tracker_id << "'" << std::endl;
    }
    
    // Make local copies of data we need to avoid holding lock
    std::vector<double> raw_yaw_log;
    std::vector<double> corrected_yaw_log;
    std::vector<std::chrono::system_clock::time_point> timestamps;
    double x, y, z, qw, qx, qy, qz;
    
    {
        std::lock_guard<std::mutex> lock(g_mutex);
        auto& tracker = g_trackers[tracker_id];
        raw_yaw_log = tracker.raw_yaw_log;
        corrected_yaw_log = tracker.corrected_yaw_log;
        timestamps = tracker.yaw_log_timestamps;
        x = tracker.x;
        y = tracker.y;
        z = tracker.z;
        qw = tracker.qw;
        qx = tracker.qx;
        qy = tracker.qy;
        qz = tracker.qz;
        
        // Clear the logs right away to avoid memory issues
        tracker.raw_yaw_log.clear();
        tracker.corrected_yaw_log.clear();
        tracker.yaw_log_timestamps.clear();
    }
    
    // Use BUILD_DATA_DIR as the definitive location
    std::string data_dir = "/home/finn/squawkblock/squawkblock/frame-calibration/build/data";
    
    // Try to create directory
    try {
        std::filesystem::create_directories(data_dir);
    } catch (const std::exception& e) {
        std::cerr << "Error creating directory: " << e.what() << std::endl;
        return;
    }
    
    // Create a unique filename with timestamp
    auto now = std::chrono::system_clock::now();
    auto now_time_t = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << data_dir << "/tracking_log_" << tracker_id << "_" 
       << std::put_time(std::localtime(&now_time_t), "%Y%m%d_%H%M%S") << ".txt";
    
    std::string filename = ss.str();
    
    // Open file for writing - abort early if it fails
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Failed to open file for writing: " << filename << std::endl;
        return;
    }
    
    // Write header
    file << "timestamp,x_pos,y_pos,z_pos,qw,qx,qy,qz,raw_yaw_rad,corrected_yaw_rad,raw_yaw_deg,corrected_yaw_deg" << std::endl;
    
    // Write data (simplified for speed)
    for (size_t i = 0; i < raw_yaw_log.size(); ++i) {
        auto time_point = timestamps[i];
        auto time_t = std::chrono::system_clock::to_time_t(time_point);
            
        // Basic data with minimal formatting
        file << std::put_time(std::localtime(&time_t), "%Y-%m-%d %H:%M:%S") << ","
             << x << "," << y << "," << z << ","
             << qw << "," << qx << "," << qy << "," << qz << ","
             << raw_yaw_log[i] << "," << corrected_yaw_log[i] << ","
             << (raw_yaw_log[i] * 180.0 / M_PI) << "," 
             << (corrected_yaw_log[i] * 180.0 / M_PI) << std::endl;
    }
    
    file.close();
    
    // Verify the file was actually created and has content
    std::ifstream verify_file(filename);
    if (verify_file.is_open()) {
        verify_file.seekg(0, std::ios::end);
        std::streamsize size = verify_file.tellg();
        verify_file.close();
        
        std::cout << "SUCCESS: Saved tracking data to " << filename << " (size: " << size << " bytes)" << std::endl;
        
        // Only try to save latest if original file was created successfully
        if (size > 0) {
            try {
                std::string latest_path = data_dir + "/latest_tracking_log.txt";
                std::ofstream latest_file(latest_path);
                if (latest_file.is_open()) {
                    // Simplest copy approach
                    std::ifstream original(filename);
                    if (original.is_open()) {
                        latest_file << original.rdbuf();
                        original.close();
                        latest_file.close();
                        std::cout << "SUCCESS: Also saved as " << latest_path << std::endl;
                    }
                }
            } catch (const std::exception& e) {
                std::cerr << "WARNING: Error saving latest file: " << e.what() << std::endl;
            }
        }
    } else {
        std::cerr << "ERROR: Failed to verify file was created: " << filename << std::endl;
        throw std::runtime_error("File could not be verified after writing");
    }
}

// Function to load drone-to-tracker mapping
void loadDroneMapping() {
    g_drones.clear();
    
    // Hardcoded mapping as requested (using Bird IDs directly as in vrpn_viz.cpp)
    g_drones.push_back({"Bird5", "Bird5", "192.168.1.107", 0.0});  // Bird5 - IP .107
    g_drones.push_back({"Bird3", "Bird3", "192.168.1.106", 0.0});  // Bird3 - IP .106
    g_drones.push_back({"Bird4", "Bird4", "192.168.1.104", 0.0});  // Bird4 - IP .104
    g_drones.push_back({"Bird1", "Bird1", "192.168.1.108", 0.0});  // Bird1 - IP .108
    
    // Load calibration data if it exists
    std::filesystem::create_directories("calibration");
    std::ifstream cal_file(CALIBRATION_FILE);
    if (cal_file.is_open()) {
        // Simple JSON parsing
        std::string line;
        std::string ip;
        double yaw_offset = 0.0;
        
        while (std::getline(cal_file, line)) {
            // Look for IP
            if (line.find("\"ip\"") != std::string::npos) {
                size_t start = line.find('"', line.find(':')) + 1;
                size_t end = line.find('"', start);
                if (start != std::string::npos && end != std::string::npos) {
                    ip = line.substr(start, end - start);
                }
            }
            
            // Look for yaw_offset
            if (line.find("\"yaw_offset\"") != std::string::npos) {
                size_t start = line.find(':', line.find("yaw_offset")) + 1;
                while (start < line.size() && (line[start] == ' ' || line[start] == '\t')) start++;
                
                size_t end = line.find(',', start);
                if (end == std::string::npos) {
                    end = line.size();
                }
                
                if (start != std::string::npos && end != std::string::npos) {
                    try {
                        yaw_offset = std::stod(line.substr(start, end - start));
                        
                        // Update the yaw_offset in the existing drone records
                        for (auto& drone : g_drones) {
                            if (drone.ip == ip) {
                                drone.yaw_offset = yaw_offset;
                                std::cout << "Loaded calibration for " << drone.name 
                                        << " (IP: " << drone.ip << "): yaw_offset = " 
                                        << yaw_offset << " degrees" << std::endl;
                                break;
                            }
                        }
                    } catch (...) {
                        std::cerr << "Error parsing yaw_offset value" << std::endl;
                    }
                }
            }
        }
    }
}

// Function to save calibration data
void saveCalibration(const std::string& ip, double yaw_offset) {
    // Create directory if it doesn't exist
    std::filesystem::create_directories("calibration");
    
    // Create a new JsonArray for our data
    JsonArray cal_data;
    
    // Get existing data first
    std::map<std::string, double> existing_data;
    
    std::ifstream in_file(CALIBRATION_FILE);
    if (in_file.is_open()) {
        std::string line;
        std::string current_ip;
        
        while (std::getline(in_file, line)) {
            // Look for IP
            if (line.find("\"ip\"") != std::string::npos) {
                size_t start = line.find('"', line.find(':')) + 1;
                size_t end = line.find('"', start);
                if (start != std::string::npos && end != std::string::npos) {
                    current_ip = line.substr(start, end - start);
                }
            }
            
            // Look for yaw_offset
            if (line.find("\"yaw_offset\"") != std::string::npos && !current_ip.empty()) {
                size_t start = line.find(':', line.find("yaw_offset")) + 1;
                while (start < line.size() && (line[start] == ' ' || line[start] == '\t')) start++;
                
                size_t end = line.find(',', start);
                if (end == std::string::npos) {
                    end = line.size();
                }
                
                if (start != std::string::npos && end != std::string::npos) {
                    try {
                        double value = std::stod(line.substr(start, end - start));
                        existing_data[current_ip] = value;
                    } catch (...) {
                        // Ignore parse errors
                    }
                }
            }
        }
        in_file.close();
    }
    
    // Update with new value
    existing_data[ip] = yaw_offset;
    
    // Create JSON array with all data
    for (const auto& [drone_ip, offset] : existing_data) {
        std::map<std::string, JsonValue> entry;
        entry["ip"] = JsonValue(drone_ip);
        entry["yaw_offset"] = JsonValue(offset);
        cal_data.push_back(entry);
    }
    
    // Save to file
    std::ofstream out_file(CALIBRATION_FILE);
    if (out_file.is_open()) {
        out_file << cal_data.dump(4); // Pretty print with 4-space indent
        std::cout << "Saved calibration data for IP: " << ip 
                << ", yaw_offset: " << yaw_offset << " degrees" << std::endl;
    } else {
        std::cerr << "Failed to open calibration file for writing" << std::endl;
    }
}

// Send command to the drone via UDP - based on swarm.cpp implementation
class TelloController {
private:
    struct TelloDevice {
        std::string ip;
        int command_port{8889};  // Default command port
        int local_port;          // Unique local port for each drone
        int command_socket{-1};
        struct sockaddr_in command_addr{};
        bool socket_valid{false};
        bool needs_reboot{true}; // Always reboot at first
        
        ~TelloDevice() {
            if (command_socket >= 0) {
                close(command_socket);
                command_socket = -1;
            }
        }
        
        bool initializeSockets() {
            // Close existing socket if any
            if (command_socket >= 0) {
                close(command_socket);
                command_socket = -1;
                socket_valid = false;
            }

            command_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
            if (command_socket < 0) {
                std::cerr << "Failed to create socket for " << ip << ": " << strerror(errno) << std::endl;
                return false;
            }

            // Duplicate the socket to ensure unique descriptor
            int new_socket = dup(command_socket);
            close(command_socket);
            command_socket = new_socket;

            std::cout << "[SOCKET] Created new socket " << command_socket 
                    << " for " << ip << ":" << local_port << std::endl;

            // Enable socket reuse
            int reuse = 1;
            if (setsockopt(command_socket, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse)) < 0) {
                std::cerr << "Failed to set SO_REUSEADDR for " << ip << ": " << strerror(errno) << std::endl;
                close(command_socket);
                return false;
            }

            // Set up local address
            struct sockaddr_in local_addr{};
            local_addr.sin_family = AF_INET;
            local_addr.sin_addr.s_addr = INADDR_ANY;
            local_addr.sin_port = htons(local_port);

            // Bind to local port
            if (bind(command_socket, (struct sockaddr *)&local_addr, sizeof(local_addr)) < 0) {
                std::cerr << "Failed to bind socket for " << ip 
                        << ": " << strerror(errno) << std::endl;
                close(command_socket);
                return false;
            }

            // Set up remote address
            command_addr = {};
            command_addr.sin_family = AF_INET;
            command_addr.sin_addr.s_addr = inet_addr(ip.c_str());
            command_addr.sin_port = htons(command_port);

            // Set socket timeout
            struct timeval tv;
            tv.tv_sec = 0;
            tv.tv_usec = 100000; // 100ms timeout
            setsockopt(command_socket, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

            socket_valid = true;
            return true;
        }
        
        bool sendCommand(const std::string& cmd) {
            if (!socket_valid || command_socket < 0) {
                std::cerr << "[ERROR] Invalid socket state for " << ip << std::endl;
                return reinitializeAndSend(cmd);
            }
            
            std::string clean_cmd = cleanCommand(cmd);
            
            std::cout << "[SEND] IP: " << ip << " Port: " << local_port 
                    << " Socket: " << command_socket 
                    << " Command: " << clean_cmd << std::endl;
            
            ssize_t sent = sendto(command_socket, clean_cmd.c_str(), clean_cmd.length(), 0,
                                (struct sockaddr *)&command_addr, sizeof(command_addr));
            
            if (sent < 0) {
                std::cerr << "Send failed for " << ip << ": " << strerror(errno) << std::endl;
                socket_valid = false;
                return reinitializeAndSend(clean_cmd);
            }
            
            std::cout << "[SUCCESS] Sent " << sent << " bytes to " << ip << std::endl;
            
            // Don't wait for responses from Tellos - they're unreliable
            // Just assume success if the send worked
            
            return true;
        }
        
        bool reinitializeAndSend(const std::string& cmd) {
            if (initializeSockets()) {
                return sendCommand(cmd);
            }
            return false;
        }
        
        std::optional<std::string> receiveResponse() {
            if (!socket_valid || command_socket < 0) return std::nullopt;
            
            // Use non-blocking receive to avoid hanging
            char buffer[1024];
            struct sockaddr_in from_addr;
            socklen_t from_len = sizeof(from_addr);
            
            // Set socket to non-blocking for this receive
            int flags = fcntl(command_socket, F_GETFL, 0);
            fcntl(command_socket, F_SETFL, flags | O_NONBLOCK);
            
            ssize_t received = recvfrom(command_socket, buffer, sizeof(buffer)-1, 0,
                                    (struct sockaddr *)&from_addr, &from_len);
            
            // Reset blocking status
            fcntl(command_socket, F_SETFL, flags);
            
            if (received < 0) {
                if (errno != EAGAIN && errno != EWOULDBLOCK) {
                    std::cerr << "Receive error for " << ip << ": " 
                            << strerror(errno) << std::endl;
                }
                return std::nullopt;
            }
            
            if (received == 0) return std::nullopt;
            
            buffer[received] = '\0';
            std::string response(buffer);
            
            std::cout << "[RESPONSE] IP: " << ip << " Response: " << response << std::endl;
            return response;
        }
        
        std::string cleanCommand(const std::string& cmd) {
            std::string clean;
            for(char c : cmd) {
                if (isalnum(c) || c == ' ' || c == '?' || c == '-') {
                    clean += c;
                }
            }
            return clean;
        }
    };
    
    std::map<std::string, TelloDevice> devices;
    int next_port = 9000;
    
public:
    // Send reboot command directly to a Tello drone (copied from swarm.cpp)
    bool rebootDrone(const std::string& ip, int port = 8889) {
        std::cout << "Rebooting drone at " << ip << "..." << std::endl;
        
        int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if (sock < 0) {
            std::cerr << "Failed to create socket for reboot: " << strerror(errno) << std::endl;
            return false;
        }

        // Set up address
        struct sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = inet_addr(ip.c_str());
        addr.sin_port = htons(port);

        // Send reboot command
        std::string cmd = "reboot";
        ssize_t sent = sendto(sock, cmd.c_str(), cmd.length(), 0, 
                          (struct sockaddr*)&addr, sizeof(addr));
        
        close(sock);
        
        if (sent < 0) {
            std::cerr << "Failed to send reboot to " << ip << ": " << strerror(errno) << std::endl;
            return false;
        }
        
        std::cout << "Sent reboot command to " << ip << std::endl;
        return true;
    }
    
    // Check if drone is responding to ping
    bool pingDrone(const std::string& ip) {
        std::string cmd = "ping -c 1 -W 1 " + ip + " > /dev/null 2>&1";
        return system(cmd.c_str()) == 0;
    }
    
    bool initialize(const std::string& ip, bool skip_reboot = false) {
        // Check if already initialized
        if (devices.find(ip) != devices.end()) {
            // If device is already initialized but needs reboot and we're not skipping it
            if (devices[ip].needs_reboot && !skip_reboot) {
                std::cout << "Rebooting drone at " << ip << " before use..." << std::endl;
                rebootDrone(ip);
                devices[ip].needs_reboot = false;
                
                // Wait for reboot to complete
                std::cout << "Waiting for drone to reboot (15 seconds)..." << std::endl;
                std::this_thread::sleep_for(std::chrono::seconds(15));
                
                // Reconnect sockets after reboot
                if (!devices[ip].initializeSockets()) {
                    std::cerr << "Failed to reinitialize sockets after reboot" << std::endl;
                    return false;
                }
            }
            return true;
        }
        
        // Skip rebooting at the start of routine
        std::cout << "Initializing drone at " << ip << " (no reboot)" << std::endl;
        
        TelloDevice device;
        device.ip = ip;
        device.local_port = next_port++;
        device.needs_reboot = false; // No need to reboot
        
        if (!device.initializeSockets()) {
            std::cerr << "Failed to initialize sockets for " << ip << std::endl;
            return false;
        }
        
        // Enter SDK mode - send the command multiple times since Tellos are unreliable
        std::cout << "Sending SDK mode command to " << ip << std::endl;
        
        // Send command 3 times for redundancy
        for (int i = 0; i < 3; i++) {
            device.sendCommand("command");
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        
        // Don't wait for response - assume it worked
        std::cout << "Assuming drone at " << ip << " entered SDK mode" << std::endl;
        devices[ip] = std::move(device);
        return true;
    }
    
    bool sendCommand(const std::string& ip, const std::string& command, bool skip_reboot = false) {
        if (devices.find(ip) == devices.end()) {
            if (!initialize(ip, skip_reboot)) {
                std::cerr << "Failed to initialize device at " << ip << " for command: " << command << std::endl;
                return false;
            }
        }
        
        bool result = devices[ip].sendCommand(command);
        if (!result) {
            std::cerr << "Failed to send command '" << command << "' to " << ip << std::endl;
        }
        return result;
    }
    
    std::optional<std::string> receiveResponse(const std::string& ip, int timeout_ms = 100) {
        if (devices.find(ip) == devices.end()) {
            return std::nullopt;
        }
        
        // For land commands, don't wait for response
        // Try once then return quickly to avoid hanging
        auto response = devices[ip].receiveResponse();
        if (response) {
            return response;
        }
        
        // For other commands, try a few times but with a short timeout
        if (timeout_ms > 0) {
            auto start = std::chrono::steady_clock::now();
            while (std::chrono::steady_clock::now() - start < std::chrono::milliseconds(timeout_ms)) {
                response = devices[ip].receiveResponse();
                if (response) {
                    return response;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        }
        
        return std::nullopt;
    }
    
    void cleanup() {
        for (auto& [ip, device] : devices) {
            if (device.socket_valid) {
                // Try to land first, then reboot
                std::cout << "Attempting to land drone at " << ip << std::endl;
                for (int i = 0; i < 3; i++) {
                    device.sendCommand("land");
                    std::this_thread::sleep_for(std::chrono::milliseconds(300));
                }
                
                // Wait for landing to complete
                std::this_thread::sleep_for(std::chrono::seconds(3));
                
                // Then reboot for clean state
                std::cout << "Rebooting drone at " << ip << " for clean exit" << std::endl;
                rebootDrone(ip);
            }
        }
        
        // Close all sockets and clear the devices map
        for (auto& [ip, device] : devices) {
            if (device.command_socket >= 0) {
                close(device.command_socket);
                device.command_socket = -1;
                device.socket_valid = false;
            }
        }
        devices.clear();
        
        std::cout << "All drones have been landed and rebooted" << std::endl;
    }
};

// Global controller instance
TelloController tello_controller;

// Send command to the drone via UDP - don't wait for responses since Tellos are unreliable
void sendDroneCommand(const std::string& ip, const std::string& command, bool skip_reboot = true) {
    std::cout << "Sending command to " << ip << ": " << command << std::endl;
    
    // Try to send the command multiple times for reliability
    bool success = false;
    for (int i = 0; i < 3; i++) {
        if (tello_controller.sendCommand(ip, command, skip_reboot)) {
            success = true;
            std::cout << "Command '" << command << "' to " << ip << " sent successfully on attempt " << (i+1) << std::endl;
            break;
        }
        std::cout << "Retrying command '" << command << "' to " << ip << "... (attempt " << (i+1) << ")" << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    if (!success) {
        std::cerr << "Failed to send command to " << ip << " after multiple attempts" << std::endl;
    }
    
    // Don't wait for responses from the drone - they're unreliable
}

// Start the calibration process for a selected drone
void startCalibration(int drone_idx) {
    if (drone_idx < 0 || drone_idx >= static_cast<int>(g_drones.size())) {
        std::cerr << "Invalid drone selection for calibration" << std::endl;
        return;
    }
    
    if (calibration_mode) {
        std::cerr << "Calibration already in progress. Please wait for it to complete." << std::endl;
        return;
    }
    
    DroneData& drone = g_drones[drone_idx];
    std::cout << "Starting calibration for " << drone.name << " (IP: " << drone.ip << ")" << std::endl;
    
    // Check if the drone is visible in OptiTrack (but don't require it)
    bool tracker_visible = false;
    {
        std::lock_guard<std::mutex> lock(g_mutex);
        const auto& tracker_it = g_trackers.find(drone.tracker_id);
        if (tracker_it != g_trackers.end()) {
            auto now = std::chrono::system_clock::now();
            auto time_diff = std::chrono::duration_cast<std::chrono::seconds>(
                now - tracker_it->second.last_update).count();
            tracker_visible = time_diff < 3 && tracker_it->second.updated;
        }
    }
    
    if (!tracker_visible) {
        std::cout << "WARNING: Drone " << drone.name << " is not currently visible in OptiTrack." << std::endl;
        std::cout << "It might become visible once it takes off from the floor." << std::endl;
    }
    
    // Clear any previous calibration data for this tracker
    {
        std::lock_guard<std::mutex> lock(g_mutex);
        if (g_trackers.find(drone.tracker_id) != g_trackers.end()) {
            g_trackers[drone.tracker_id].yaw_values.clear();
            g_trackers[drone.tracker_id].yaw_timestamps.clear();
            
            // Reset yaw logs for new calibration
            g_trackers[drone.tracker_id].raw_yaw_log.clear();
            g_trackers[drone.tracker_id].corrected_yaw_log.clear();
            g_trackers[drone.tracker_id].yaw_log_timestamps.clear();
            
            // Reset flip detection state
            g_trackers[drone.tracker_id].has_prior_yaw = false;
        }
    }
    
    // Make sure we can communicate with the drone
    if (!tello_controller.initialize(drone.ip)) {
        std::cerr << "ERROR: Failed to initialize connection to drone at " << drone.ip << std::endl;
        return;
    }
    
    // Set flags for calibration mode
    calibration_mode = true;
    calibration_start_time = std::chrono::system_clock::now();
    
    // Create a thread for the timed command sequence
    std::thread([drone]() {
        try {
            std::cout << "CALIBRATION: Step 1 - Taking off and waiting 3 seconds" << std::endl;
            
            // 1. Takeoff and hover for 3 seconds - send command multiple times for reliability
            for (int i = 0; i < 3; i++) {
                tello_controller.sendCommand(drone.ip, "takeoff");
                std::this_thread::sleep_for(std::chrono::milliseconds(300));
            }
            
            // Don't wait for a response, just wait a bit to make sure the takeoff happens
            std::cout << "Waiting for drone to take off..." << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(3));
            
            // Clear any existing data for this tracker
            {
                std::lock_guard<std::mutex> lock(g_mutex);
                const auto& tracker_it = g_trackers.find(drone.tracker_id);
                if (tracker_it != g_trackers.end()) {
                    // Clear previous tracking data to ensure we only get fresh data
                    tracker_it->second.raw_yaw_log.clear();
                    tracker_it->second.corrected_yaw_log.clear();
                    tracker_it->second.yaw_log_timestamps.clear();
                    tracker_it->second.yaw_values.clear();
                    tracker_it->second.yaw_timestamps.clear();
                    
                    // Check if tracker is currently visible (for debug purposes only)
                    auto now = std::chrono::system_clock::now();
                    auto time_diff = std::chrono::duration_cast<std::chrono::seconds>(
                        now - tracker_it->second.last_update).count();
                    
                    if (time_diff < 3 && tracker_it->second.updated) {
                        std::cout << "INFO: Drone is currently visible in OptiTrack" << std::endl;
                    } else {
                        std::cout << "INFO: Drone is not currently visible in OptiTrack, but continuing anyway" << std::endl;
                        std::cout << "      It should become visible once it gains altitude" << std::endl;
                    }
                } else {
                    std::cout << "WARNING: Could not find tracker '" << drone.tracker_id 
                              << "' in tracker list - it may not be registered properly" << std::endl;
                }
            }
            
            // Record the takeoff time - do this after visibility check
            takeoff_time = std::chrono::system_clock::now();
            drone_flying = true;
            
            // Also print debug info about the tracker
            {
                std::lock_guard<std::mutex> lock(g_mutex);
                const auto& tracker_it = g_trackers.find(drone.tracker_id);
                if (tracker_it != g_trackers.end()) {
                    std::cout << "DEBUG: Tracker data - "
                              << "Pos: (" << tracker_it->second.x << ", " 
                              << tracker_it->second.y << ", " 
                              << tracker_it->second.z << ") " 
                              << "Updated: " << (tracker_it->second.updated ? "YES" : "NO")
                              << std::endl;
                } else {
                    std::cout << "DEBUG: Tracker '" << drone.tracker_id << "' not found in g_trackers!" << std::endl;
                }
            }
            
            // Move upwards for 3 seconds to gain more height
            std::cout << "CALIBRATION: Moving upward for 3 seconds to gain height" << std::endl;
            for (int i = 0; i < 3; i++) {
                tello_controller.sendCommand(drone.ip, "rc 0 0 50 0");  // rc left-right forward-back up-down yaw
                std::this_thread::sleep_for(std::chrono::milliseconds(300));
            }
            
            // Keep moving up for 3 seconds
            std::this_thread::sleep_for(std::chrono::seconds(3));
            
            // Stop upward movement
            for (int i = 0; i < 2; i++) {
                tello_controller.sendCommand(drone.ip, "rc 0 0 0 0");
                std::this_thread::sleep_for(std::chrono::milliseconds(300));
            }
            
            // Wait a second to stabilize
            std::this_thread::sleep_for(std::chrono::seconds(1));
            
            // 2. Move forward for 5 seconds
            std::cout << "CALIBRATION: Step 2 - Moving forward for 5 seconds" << std::endl;
            
            // This direction will be our reference for "forward" in the drone's command frame
            // We'll use yaw = 0 for this direction
            calibration_drone_yaw = 0.0;  // By definition, this is 0 in drone's frame
            
            // Send the command to move forward at 0.5 m/s (using rc command for more control)
            // Send multiple times to ensure it's received
            for (int i = 0; i < 5; i++) {
                tello_controller.sendCommand(drone.ip, "rc 0 50 0 0");  // rc lr fb ud yaw
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
            
            // Sleep for 5 seconds while drone moves forward
            std::cout << "Moving forward for 5 seconds..." << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(5));
            
            // 3. Stop and land
            std::cout << "CALIBRATION: Step 3 - Stopping and landing" << std::endl;
            
            // Send stop command multiple times
            for (int i = 0; i < 3; i++) {
                tello_controller.sendCommand(drone.ip, "rc 0 0 0 0");
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
            
            std::this_thread::sleep_for(std::chrono::seconds(1));
            
            // Send land command once - no waiting
            tello_controller.sendCommand(drone.ip, "land");
            std::cout << "Landing command sent." << std::endl;
            
            // First save the log data WITHOUT holding the mutex
            // Get a snapshot of the data we need
            std::vector<double> raw_yaw_log_copy;
            std::vector<double> corrected_yaw_log_copy;
            std::vector<std::chrono::system_clock::time_point> timestamps_copy;
            double pos_x = 0, pos_y = 0, pos_z = 0;
            double orient_qw = 1.0, orient_qx = 0, orient_qy = 0, orient_qz = 0;
            bool has_data_to_save = false;
            
            // Make a copy of the data with minimal time holding the lock
            {
                std::lock_guard<std::mutex> lock(g_mutex);
                if (g_trackers.find(drone.tracker_id) != g_trackers.end()) {
                    auto& tracker = g_trackers[drone.tracker_id];
                    
                    // Copy the data we need
                    raw_yaw_log_copy = tracker.raw_yaw_log;
                    corrected_yaw_log_copy = tracker.corrected_yaw_log;
                    timestamps_copy = tracker.yaw_log_timestamps;
                    pos_x = tracker.x;
                    pos_y = tracker.y;
                    pos_z = tracker.z;
                    orient_qw = tracker.qw;
                    orient_qx = tracker.qx;
                    orient_qy = tracker.qy;
                    orient_qz = tracker.qz;
                    
                    // Clear the logs to prevent duplicate saving
                    has_data_to_save = !tracker.raw_yaw_log.empty();
                    tracker.raw_yaw_log.clear();
                    tracker.corrected_yaw_log.clear();
                    tracker.yaw_log_timestamps.clear();
                }
            }
            
            // Save the data without holding the mutex
            if (has_data_to_save) {
                std::cout << "INFO: Preparing to save " << raw_yaw_log_copy.size() << " data points" << std::endl;
                
                try {
                    // Write the data to file
                    std::string data_dir = "/home/finn/squawkblock/squawkblock/frame-calibration/build/data";
                    std::filesystem::create_directories(data_dir);
                    
                    // Create a unique filename with timestamp
                    auto now = std::chrono::system_clock::now();
                    auto now_time_t = std::chrono::system_clock::to_time_t(now);
                    std::stringstream ss;
                    ss << data_dir << "/tracking_log_" << drone.tracker_id << "_" 
                       << std::put_time(std::localtime(&now_time_t), "%Y%m%d_%H%M%S") << ".txt";
                    
                    std::string filename = ss.str();
                    
                    std::ofstream file(filename);
                    if (file.is_open()) {
                        // Write header
                        file << "timestamp,x_pos,y_pos,z_pos,qw,qx,qy,qz,raw_yaw_rad,corrected_yaw_rad,raw_yaw_deg,corrected_yaw_deg" << std::endl;
                        
                        // Write data
                        for (size_t i = 0; i < raw_yaw_log_copy.size(); ++i) {
                            auto time_point = timestamps_copy[i];
                            auto time_t = std::chrono::system_clock::to_time_t(time_point);
                                
                            // Basic data with minimal formatting
                            file << std::put_time(std::localtime(&time_t), "%Y-%m-%d %H:%M:%S") << ","
                                 << pos_x << "," << pos_y << "," << pos_z << ","
                                 << orient_qw << "," << orient_qx << "," << orient_qy << "," << orient_qz << ","
                                 << raw_yaw_log_copy[i] << "," << corrected_yaw_log_copy[i] << ","
                                 << (raw_yaw_log_copy[i] * 180.0 / M_PI) << "," 
                                 << (corrected_yaw_log_copy[i] * 180.0 / M_PI) << std::endl;
                        }
                        
                        file.close();
                        std::cout << "SUCCESS: Saved tracking data to " << filename << std::endl;
                        
                        // Also make a latest copy
                        std::string latest_path = data_dir + "/latest_tracking_log.txt";
                        std::ofstream latest_file(latest_path);
                        if (latest_file.is_open()) {
                            std::ifstream original(filename);
                            if (original.is_open()) {
                                latest_file << original.rdbuf();
                                original.close();
                                latest_file.close();
                            }
                        }
                    } else {
                        std::cerr << "ERROR: Failed to open file for writing: " << filename << std::endl;
                    }
                } catch (const std::exception& e) {
                    std::cerr << "Error saving log file: " << e.what() << " (continuing anyway)" << std::endl;
                }
            } else {
                std::cout << "WARNING: No tracking data to save for " << drone.tracker_id << std::endl;
            }
            
            // Now calculate the calibration with a fresh mutex lock
            {
                std::lock_guard<std::mutex> lock(g_mutex);
                if (g_trackers.find(drone.tracker_id) != g_trackers.end()) {
                    // Calculate average yaw from recorded values
                    auto& tracker = g_trackers[drone.tracker_id];
                    
                    if (!tracker.yaw_values.empty()) {
                        // Calculate average yaw in OptiTrack frame
                        double sum_sin = 0, sum_cos = 0;
                        for (double yaw : tracker.yaw_values) {
                            sum_sin += std::sin(yaw);
                            sum_cos += std::cos(yaw);
                        }
                        double avg_yaw = std::atan2(sum_sin / tracker.yaw_values.size(), 
                                                  sum_cos / tracker.yaw_values.size());
                        
                        // Calculate yaw offset - this is the difference between drone's command frame
                        // and OptiTrack frame
                        double yaw_offset = (avg_yaw - calibration_drone_yaw) * 180.0 / M_PI;
                        
                        // Normalize to [-180, 180]
                        while (yaw_offset > 180.0) yaw_offset -= 360.0;
                        while (yaw_offset < -180.0) yaw_offset += 360.0;
                        
                        // Update drone data
                        for (auto& d : g_drones) {
                            if (d.ip == drone.ip) {
                                d.yaw_offset = yaw_offset;
                                break;
                            }
                        }
                        
                        // Save to file
                        saveCalibration(drone.ip, yaw_offset);
                        
                        std::cout << "\n========================================\n";
                        std::cout << "CALIBRATION COMPLETE FOR " << drone.name << " (IP: " << drone.ip << ")\n";
                        std::cout << "Yaw offset: " << yaw_offset << " degrees (+ve is clockwise from ceiling)\n";
                        std::cout << "Calibration data points: " << tracker.yaw_values.size() << "\n";
                        std::cout << "Tracking data points: " << tracker.raw_yaw_log.size() << "\n";
                        std::cout << "Full tracking log is saved in the build/data directory\n";
                        std::cout << "The drone will now correctly translate commands to the OptiTrack coordinate system\n";
                        std::cout << "========================================\n";
                    } else {
                        std::cerr << "ERROR: No yaw data collected during calibration! Make sure the drone's tracker is visible in OptiTrack" << std::endl;
                    }
                } else {
                    std::cerr << "ERROR: Tracker data not found for " << drone.tracker_id << std::endl;
                }
                
                calibration_mode = false;
                drone_flying = false;
            }
            
            // Return to main menu
            std::cout << "Returning to main menu...\n";
            printMenu();
        } catch (const std::exception& e) {
            std::cerr << "ERROR during calibration: " << e.what() << std::endl;
            
            // Try to save log data, but don't worry if it fails
            try {
                saveYawLogToFile(drone.tracker_id);
            } catch (const std::exception&) {
                // Ignore errors during emergency landing
            }
            
            // Attempt emergency landing - don't wait for confirmation
            std::cout << "Attempting emergency landing..." << std::endl;
            for (int i = 0; i < 3; i++) {
                tello_controller.sendCommand(drone.ip, "land");
                std::this_thread::sleep_for(std::chrono::milliseconds(300));
            }
            std::cout << "Emergency land command sent" << std::endl;
            
            // Set global state flags
            {
                std::lock_guard<std::mutex> lock(g_mutex);
                calibration_mode = false;
                drone_flying = false;
            }
            
            // Don't wait for response
            
            // Return to main menu even after error
            std::cout << "Returning to main menu after error...\n";
            printMenu();
        }
    }).detach();
}

// Print menu options
void printMenu() {
    std::cout << "\n=== Drone Frame Calibration Tool ===\n";
    std::cout << "Select an option:\n";
    std::cout << "1. List online drones\n";
    std::cout << "2. Network scan (not implemented)\n";
    std::cout << "3. Map OptiTrack to drones (not implemented)\n";
    std::cout << "4. Calibrate drone orientation\n";
    std::cout << "5. Reboot all drones\n";
    std::cout << "6. Exit\n";
    std::cout << "Enter choice: ";
}

// Function to reboot all drones
void rebootAllDrones() {
    std::cout << "Rebooting all drones..." << std::endl;
    
    for (const auto& drone : g_drones) {
        std::cout << "Rebooting " << drone.name << " (IP: " << drone.ip << ")" << std::endl;
        tello_controller.rebootDrone(drone.ip);
    }
    
    std::cout << "Reboot commands sent to all drones." << std::endl;
    std::cout << "Waiting 15 seconds for reboots to complete..." << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(15));
    std::cout << "Reboot process completed." << std::endl;
}

// Signal handler for clean exits
void signalHandler(int signal) {
    std::cout << "Received signal " << signal << ", cleaning up..." << std::endl;
    tello_controller.cleanup();
    exit(signal);
}

int main() {
    // Register signal handlers for clean shutdown
    signal(SIGINT, signalHandler);   // Ctrl+C
    signal(SIGTERM, signalHandler);  // Kill signal
    
    // Create visualization window
    cv::namedWindow("Tracker Visualization", cv::WINDOW_AUTOSIZE);
    cv::Mat display(800, 1000, CV_8UC3);  // Larger display
    
    // Load drone-to-tracker mapping
    loadDroneMapping();

    // Predefined colors
    std::vector<cv::Scalar> colors = {
        cv::Scalar(255, 0, 0),   // Blue
        cv::Scalar(0, 255, 0),   // Green
        cv::Scalar(0, 0, 255),   // Red
        cv::Scalar(255, 255, 0), // Cyan
        cv::Scalar(255, 0, 255), // Magenta
        cv::Scalar(0, 255, 255), // Yellow
        cv::Scalar(128, 0, 0),   // Dark Blue
        cv::Scalar(0, 128, 0),   // Dark Green
        cv::Scalar(0, 0, 128)    // Dark Red
    };

    std::vector<std::pair<std::string*, vrpn_Tracker_Remote*>> trackers;
    int color_idx = 0;

    // Initialize VRPN trackers - using exact same code as vrpn_viz.cpp
    auto add_tracker = [&](const std::string& base_name, int idx) {
        std::string name = base_name + std::to_string(idx);
        // Connect to trackers, no console output
        
        auto tracker_name = new std::string(name);
        auto tracker = new vrpn_Tracker_Remote((name + "@192.168.1.100:3883").c_str());
        
        // Disable VRPN's error logging (redirect to nowhere)
        // This prevents messages like "Trying to reconnect" from appearing in the console
        tracker->shutup = true;
        
        tracker->register_change_handler(tracker_name, handle_tracker);
        trackers.push_back({tracker_name, tracker});
        
        // Initialize tracker with empty path
        TrackerData td;
        td.color = colors[color_idx++ % colors.size()];
        td.updated = false;
        g_trackers[name] = td;
    };

    // Add exactly the same trackers as in vrpn_viz.cpp
    for (int i = 1; i <= 3; i++) {
        add_tracker("Tracker", i);
    }
    for (int i = 1; i <= 6; i++) {
        add_tracker("Bird", i);
    }
    
    // Menu handling in a separate thread
    std::thread menu_thread([]() {
        while (true) {
            printMenu();
            int choice;
            std::cin >> choice;
            
            // First handle special cases
            if (choice == 5) {
                // Reboot all drones
                rebootAllDrones();
                continue; // Go back to menu
            }
            else if (choice == 6) {
                std::cout << "Cleaning up and exiting..." << std::endl;
                tello_controller.cleanup();
                exit(0);
            }
            
            switch (choice) {
                case 1: {
                    // List drones and their statuses based on network connectivity
                    std::cout << "\nAvailable drones:\n";
                    for (size_t i = 0; i < g_drones.size(); i++) {
                        const auto& drone = g_drones[i];
                        
                        // By default, assume all drones from our mapping are online
                        // We'll add ping checks in the future
                        bool is_online = true;
                        
                        // Check if drone has a calibrated yaw offset
                        std::string calibration_status = "";
                        if (drone.yaw_offset != 0.0) {
                            calibration_status = " - Yaw offset: " + std::to_string(drone.yaw_offset) + " degrees";
                        } else {
                            calibration_status = " - Not calibrated";
                        }
                        
                        // Check if tracker is visible in OptiTrack
                        bool tracker_visible = false;
                        {
                            std::lock_guard<std::mutex> lock(g_mutex);
                            const auto& tracker_it = g_trackers.find(drone.tracker_id);
                            if (tracker_it != g_trackers.end()) {
                                auto now = std::chrono::system_clock::now();
                                auto time_diff = std::chrono::duration_cast<std::chrono::seconds>(
                                    now - tracker_it->second.last_update).count();
                                tracker_visible = time_diff < 3 && tracker_it->second.updated;
                            }
                        }
                        
                        std::cout << i+1 << ". " << drone.name << " (IP: " << drone.ip 
                                << ", Tracker: " << drone.tracker_id << ")"
                                << "\n   Network: " << (is_online ? "ONLINE" : "OFFLINE")
                                << "  |  OptiTrack: " << (tracker_visible ? "VISIBLE" : "NOT VISIBLE")
                                << calibration_status << std::endl;
                    }
                    break;
                }
                case 2:
                    std::cout << "Network scan is not implemented yet.\n";
                    break;
                case 3:
                    std::cout << "Manual OptiTrack mapping is not implemented yet.\n";
                    break;
                case 4: {
                    // Calibrate a specific drone
                    if (calibration_mode) {
                        std::cout << "Calibration already in progress. Please wait.\n";
                        break;
                    }
                    
                    std::cout << "Select drone to calibrate (1-" << g_drones.size() << "): ";
                    int drone_idx;
                    std::cin >> drone_idx;
                    
                    if (drone_idx < 1 || drone_idx > static_cast<int>(g_drones.size())) {
                        std::cout << "Invalid selection.\n";
                        break;
                    }
                    
                    // Start calibration for selected drone
                    startCalibration(drone_idx - 1);  // Convert to 0-based
                    break;
                }
                case 5:
                    std::cout << "Exiting...\n";
                    exit(0);
                    break;
                default:
                    std::cout << "Invalid choice. Try again.\n";
            }
        }
    });
    menu_thread.detach();

    // Main visualization loop
    while (true) {
        for (auto& t : trackers) {
            t.second->mainloop();
        }

        // Clear display
        display = cv::Scalar(0, 0, 0);

        // Draw grid
        cv::Point center(display.cols/2, display.rows/2);
        const int grid_size = 10;
        
        // Draw coordinate grid
        for (int i = -grid_size; i <= grid_size; i++) {
            // Vertical lines
            cv::Point p1 = worldToScreen(i, -grid_size, center);
            cv::Point p2 = worldToScreen(i, grid_size, center);
            cv::line(display, p1, p2, cv::Scalar(50, 50, 50), 1);
            
            // Horizontal lines
            p1 = worldToScreen(-grid_size, i, center);
            p2 = worldToScreen(grid_size, i, center);
            cv::line(display, p1, p2, cv::Scalar(50, 50, 50), 1);
        }

        // Draw axes
        cv::line(display, center, worldToScreen(grid_size, 0, center), cv::Scalar(0, 0, 255), 2);  // X axis
        cv::line(display, center, worldToScreen(0, grid_size, center), cv::Scalar(0, 255, 0), 2);  // Y axis
        
        cv::putText(display, "X", worldToScreen(grid_size + 0.5, 0, center), 
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255));
        cv::putText(display, "Y", worldToScreen(0, grid_size + 0.5, center), 
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0));

        // Draw status text
        int text_y = 20;
        cv::putText(display, "Tracking Status:", cv::Point(10, text_y), 
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255));
        text_y += 20;

        // Draw calibration status if active
        if (calibration_mode) {
            cv::putText(display, "CALIBRATION IN PROGRESS", cv::Point(10, text_y), 
                      cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255));
            text_y += 20;
        }

        // Draw trackers
        {
            std::lock_guard<std::mutex> lock(g_mutex);
            auto now = std::chrono::system_clock::now();
            
            // Find which trackers correspond to drones
            std::map<std::string, std::string> tracker_to_drone;
            for (const auto& drone : g_drones) {
                tracker_to_drone[drone.tracker_id] = drone.name;
            }
            
            // Collect and sort tracker names
            std::vector<std::string> tracker_names;
            for (const auto& t : g_trackers) {
                tracker_names.push_back(t.first);
            }
            std::sort(tracker_names.begin(), tracker_names.end());
            
            // Display status and visuals for each tracker
            for (const auto& name : tracker_names) {
                const auto& t = g_trackers[name];
                auto time_diff = std::chrono::duration_cast<std::chrono::seconds>(
                    now - t.last_update).count();
                bool is_active = time_diff < 3 && t.updated;
                
                // Only show trackers that map to drones
                if (tracker_to_drone.find(name) != tracker_to_drone.end()) {
                    std::string drone_name = tracker_to_drone[name];
                    
                    // Status color based on activity
                    cv::Scalar status_color = is_active ? 
                        cv::Scalar(0, 255, 0) : cv::Scalar(128, 128, 128);
                    
                    // Status text
                    std::string status_text = drone_name + " (" + name + "): " + 
                        (is_active ? "Online" : "Offline");
                    
                    cv::putText(display, status_text, cv::Point(10, text_y), 
                              cv::FONT_HERSHEY_SIMPLEX, 0.5, status_color);
                    text_y += 20;
                    
                    // Draw tracker visualization if active
                    if (is_active) {
                        // Draw position
                        cv::Point pos = worldToScreen(t.x, t.y, center);
                        
                        // Draw path history
                        if (t.path.size() > 1) {
                            for (size_t i = 1; i < t.path.size(); i++) {
                                // Calculate alpha for fade effect
                                double alpha = std::min(1.0, 0.3 + 0.7 * (static_cast<double>(i) / t.path.size()));
                                cv::Scalar pathColor = t.color * alpha;
                                
                                cv::line(display, t.path[i-1], t.path[i], pathColor, 1);
                            }
                        }
                        
                        // Draw orientation arrow showing yaw
                        double raw_yaw = quaternionToYaw(t.qw, t.qx, t.qy, t.qz);
                        
                        // Apply flip correction
                        double corrected_yaw = raw_yaw;
                        if (t.has_prior_yaw) {
                            corrected_yaw = correctYawFlip(raw_yaw, t.last_corrected_yaw);
                        }
                        
                        // Find corresponding drone for yaw offset
                        double yaw_offset = 0.0;
                        for (const auto& drone : g_drones) {
                            if (drone.tracker_id == name) {
                                yaw_offset = drone.yaw_offset * M_PI / 180.0;  // Convert to radians
                                break;
                            }
                        }
                        
                        // Apply calibration offset to the corrected yaw
                        double adjusted_yaw = corrected_yaw - yaw_offset;
                        
                        // Calculate direction vector
                        double vx = std::cos(adjusted_yaw);
                        double vy = std::sin(adjusted_yaw);
                        
                        // Draw both arrows - red for uncalibrated, green for calibrated
                        double length = 30.0;
                        
                        // Raw uncorrected yaw (red) - shows OptiTrack's raw reading
                        cv::Point rawArrowEnd(pos.x + static_cast<int>(std::cos(raw_yaw) * length), 
                                            pos.y - static_cast<int>(std::sin(raw_yaw) * length));
                        cv::arrowedLine(display, pos, rawArrowEnd, cv::Scalar(0, 0, 255), 1, cv::LINE_AA, 0, 0.3);
                        
                        // Flip-corrected but not calibrated (blue) - shows corrected reading
                        cv::Point correctedArrowEnd(pos.x + static_cast<int>(std::cos(corrected_yaw) * length), 
                                                   pos.y - static_cast<int>(std::sin(corrected_yaw) * length));
                        cv::arrowedLine(display, pos, correctedArrowEnd, cv::Scalar(255, 0, 0), 2, cv::LINE_AA, 0, 0.3);
                        
                        // Calibrated arrow (green)
                        cv::Point calArrowEnd(pos.x + static_cast<int>(vx * length), 
                                            pos.y - static_cast<int>(vy * length));
                        cv::arrowedLine(display, pos, calArrowEnd, cv::Scalar(0, 255, 0), 2, cv::LINE_AA, 0, 0.3);
                        
                        // Draw position marker
                        cv::circle(display, pos, 5, t.color, -1);
                        
                        // Draw info text
                        // Show yaw angles for debugging
                        std::string pos_text = drone_name + " Z:" + std::to_string(t.z);
                        cv::putText(display, pos_text, pos + cv::Point(10, 10), 
                                  cv::FONT_HERSHEY_SIMPLEX, 0.5, t.color);
                                  
                        // Display raw and corrected yaw values for debugging
                        int raw_yaw_deg = static_cast<int>(raw_yaw * 180.0 / M_PI);
                        int corrected_yaw_deg = static_cast<int>(corrected_yaw * 180.0 / M_PI);
                        std::string yaw_text = "Raw: " + std::to_string(raw_yaw_deg) + "° Cor: " + std::to_string(corrected_yaw_deg) + "°";
                        cv::putText(display, yaw_text, pos + cv::Point(10, 25), 
                                   cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255));
                    }
                }
            }
        }

        cv::imshow("Tracker Visualization", display);
        
        char key = cv::waitKey(1);
        if (key == 27) // ESC
            break;
        else if (key == '+' || key == '=') {
            // Increase scale
            const_cast<double&>(SCALE) *= 1.1;
        }
        else if (key == '-' || key == '_') {
            // Decrease scale
            const_cast<double&>(SCALE) /= 1.1;
        }
        else if (key == 'c' || key == 'C') {
            // Clear all paths
            std::lock_guard<std::mutex> lock(g_mutex);
            for (auto& tracker : g_trackers) {
                tracker.second.path.clear();
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // Cleanup
    for (auto& t : trackers) {
        delete t.first;
        delete t.second;
    }

    return 0;
}