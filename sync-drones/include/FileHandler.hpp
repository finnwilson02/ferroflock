// include/FileHandler.hpp
#pragma once
#include <string>
#include <vector>
#include <fstream>
#include <iostream>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <cstdlib>
#include <regex>
#include <sstream>
#include <array>
#include <functional>
#include <filesystem>
#include <unordered_map>
#include <optional>
#include <chrono>
#include <poll.h>
#include <cstring>  // For strerror
#include <cerrno>   // For errno

// Structure to store VRPN tracker data
struct VrpnData {
    std::array<float, 3> position;  // x, y, z
    std::array<float, 4> rotation;  // qw, qx, qy, qz (quaternion)
};

struct DroneConfig {
    std::string mac;
    std::string ip;
    std::string tracker_id;   // VRPN tracker ID
    float pitch_bias{0};
    float roll_bias{0};
    float yaw_bias{0};        // Added yaw bias
    bool is_flying{false};
    bool is_connected{false}; // Flag to indicate if drone is responsive
    bool is_online{false};    // Flag to indicate if drone was found in network scan
    int command_socket{-1};
    struct sockaddr_in command_addr{};
    std::chrono::steady_clock::time_point flight_start_time; // Track flight time
    std::string last_seen;    // Timestamp when drone was last seen
};

class FileHandler {
private:
    static std::string calibration_dir;
    static std::string mapping_file;
    static std::string calibration_file;

public:
    FileHandler() {
        // Create calibration directory if it doesn't exist
        std::filesystem::create_directories(calibration_dir);
    }

    static std::vector<DroneConfig> findDrones(bool scan_network = false) {
        std::vector<DroneConfig> drones;
        
        if (scan_network) {
            std::cout << "Scanning network for DJI drones (MAC prefix 34:D2:62)..." << std::endl;
            
            // Run the Python script to find drones and generate dji_devices.json
            std::string python_script = "/home/finn/squawkblock/squawkblock/sync-drones/python/get_devices.py";
            
            // Check if script exists
            if (!std::filesystem::exists(python_script)) {
                // Try relative path
                python_script = "../python/get_devices.py";
                if (!std::filesystem::exists(python_script)) {
                    python_script = "python/get_devices.py";
                    if (!std::filesystem::exists(python_script)) {
                        std::cerr << "Could not find get_devices.py script" << std::endl;
                        return drones;
                    }
                }
            }
            
            // Run the Python script to scan for drones
            std::cout << "Running " << python_script << " to scan for drones..." << std::endl;
            int result = std::system(("python3 " + python_script).c_str());
            
            if (result != 0) {
                std::cerr << "Failed to run drone scanning script. Error code: " << result << std::endl;
                std::cerr << "Please run python/get_devices.py manually" << std::endl;
            }
        } else {
            std::cout << "Loading drone data from existing configuration..." << std::endl;
        }
        
        // Try different paths for dji_devices.json - only look in main directory to avoid duplicates
        std::string paths[] = {
            "dji_devices.json",                                          // Current directory
            "../dji_devices.json"                                        // Parent directory 
        };
        
        std::ifstream file;
        std::string found_path;
        
        for (const auto& path : paths) {
            file.open(path);
            if (file.is_open()) {
                found_path = path;
                break;
            }
        }
        
        if (!file.is_open()) {
            std::cerr << "Could not open dji_devices.json - scan may have failed" << std::endl;
            return drones;
        }
        
        std::cout << "Found device file at: " << found_path << std::endl;
        
        // Read the entire file content
        std::string content((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
        file.close();
        
        // Parse JSON manually to get both online and offline drones
        int online_count = 0;
        int offline_count = 0;
        size_t pos = 0;
        
        while (true) {
            size_t obj_start = content.find("{", pos);
            if (obj_start == std::string::npos) break;
            
            size_t obj_end = content.find("}", obj_start);
            if (obj_end == std::string::npos) break;
            
            std::string obj = content.substr(obj_start, obj_end - obj_start + 1);
            
            // Extract IP
            std::string ip;
            size_t ip_pos = obj.find("\"ip\":");
            if (ip_pos != std::string::npos) {
                size_t ip_start = obj.find("\"", ip_pos + 5) + 1;
                size_t ip_end = obj.find("\"", ip_start);
                ip = obj.substr(ip_start, ip_end - ip_start);
            }
            
            // Extract MAC
            std::string mac;
            size_t mac_pos = obj.find("\"mac\":");
            if (mac_pos != std::string::npos) {
                size_t mac_start = obj.find("\"", mac_pos + 6) + 1;
                size_t mac_end = obj.find("\"", mac_start);
                mac = obj.substr(mac_start, mac_end - mac_start);
            }
            
            // Check if drone is online
            bool is_online = false;
            size_t online_pos = obj.find("\"online\":");
            if (online_pos != std::string::npos) {
                std::string online_value = obj.substr(online_pos + 9, 5); // true or false
                is_online = (online_value.find("true") != std::string::npos);
            }
            
            // Extract last seen timestamp
            std::string last_seen;
            size_t last_seen_pos = obj.find("\"last_seen\":");
            if (last_seen_pos != std::string::npos) {
                size_t last_seen_start = obj.find("\"", last_seen_pos + 12) + 1;
                size_t last_seen_end = obj.find("\"", last_seen_start);
                last_seen = obj.substr(last_seen_start, last_seen_end - last_seen_start);
            }
            
            // Create drone config if we have both IP and MAC
            if (!ip.empty() && !mac.empty()) {
                DroneConfig drone;
                drone.ip = ip;
                drone.mac = mac;
                drone.is_online = is_online;
                drone.last_seen = last_seen;
                
                // Only try to communicate with online drones
                drone.is_connected = is_online;
                
                drones.push_back(drone);
                
                if (is_online) {
                    std::cout << "Found online drone: IP=" << drone.ip 
                              << " MAC=" << drone.mac << std::endl;
                    online_count++;
                } else {
                    std::cout << "Found offline drone: MAC=" << drone.mac 
                              << " Last IP=" << drone.ip
                              << " Last seen=" << drone.last_seen << std::endl;
                    offline_count++;
                }
            }
            
            // Move past this entry for next search
            pos = obj_end + 1;
        }
        
        if (drones.empty()) {
            std::cout << "No drones found. Make sure drones are powered on and connected to the network." << std::endl;
        } else {
            std::cout << "Found " << online_count << " online and " << offline_count 
                      << " offline drones (total: " << drones.size() << ")" << std::endl;
        }

        return drones;
    }

    static void loadCalibration(std::vector<DroneConfig>& drones) {
        // Create directory if it doesn't exist
        std::filesystem::create_directories(calibration_dir);
        
        // First load mappings (tracker IDs)
        loadMapping(drones);
        
        // Then load bias values
        std::ifstream file(calibration_dir + "/" + calibration_file);
        if (!file.is_open()) {
            std::cout << "No existing calibration found at " << calibration_dir + "/" + calibration_file << std::endl;
            return;
        }

        std::string line;
        while (std::getline(file, line)) {
            std::istringstream iss(line);
            std::string mac;
            float pitch, roll, yaw;
            char comma;  // To consume the commas

            if (std::getline(iss, mac, ',')) {
                iss >> pitch >> comma >> roll >> comma;
                
                // Check if yaw is present (backward compatibility)
                if (iss >> yaw) {
                    // Got yaw value
                } else {
                    yaw = 0.0f;  // Default if not present
                }
                
                for (auto& drone : drones) {
                    if (drone.mac == mac) {
                        drone.pitch_bias = pitch;
                        drone.roll_bias = roll;
                        drone.yaw_bias = yaw;
                        std::cout << "Loaded calibration for " << mac 
                                << ": pitch=" << pitch << " roll=" << roll
                                << " yaw=" << yaw << std::endl;
                        break;
                    }
                }
            }
        }
    }

    static void loadMapping(std::vector<DroneConfig>& drones) {
        std::ifstream file(calibration_dir + "/" + mapping_file);
        if (!file.is_open()) {
            std::cout << "No existing tracker mapping found" << std::endl;
            return;
        }

        std::string line;
        while (std::getline(file, line)) {
            std::istringstream iss(line);
            std::string mac, tracker_id;
            char comma;  // To consume the comma

            if (std::getline(iss, mac, ',') && std::getline(iss, tracker_id)) {
                for (auto& drone : drones) {
                    if (drone.mac == mac) {
                        drone.tracker_id = tracker_id;
                        std::cout << "Loaded tracker mapping: " << mac 
                                << " -> " << tracker_id << std::endl;
                        break;
                    }
                }
            }
        }
    }

    static void saveMapping(const std::vector<DroneConfig>& drones) {
        std::cout << "CRITICAL: Starting to save drone-to-tracker mapping..." << std::endl;
        
        // Create directory if it doesn't exist - force absolute path
        std::string abs_calibration_dir = "/home/finn/squawkblock/squawkblock/sync-drones/build/calibration";
        std::filesystem::create_directories(abs_calibration_dir);
        
        // Use absolute path for mapping file
        std::string file_path = abs_calibration_dir + "/" + mapping_file;
        std::cout << "CRITICAL: Using absolute path for mapping file: " << file_path << std::endl;
        
        // Try multiple ways to open the file for writing
        std::ofstream file;
        
        // Method 1: std::ios::out | std::ios::trunc
        file.open(file_path, std::ios::out | std::ios::trunc);
        if (!file.is_open()) {
            std::cerr << "Method 1 failed to open file for writing." << std::endl;
            
            // Method 2: C-style FILE* and then convert to ofstream
            FILE* fp = fopen(file_path.c_str(), "w");
            if (fp != nullptr) {
                fclose(fp);
                file.open(file_path, std::ios::out);
            } else {
                std::cerr << "Method 2 failed. Error: " << strerror(errno) << std::endl;
            }
        }
        
        if (!file.is_open()) {
            std::cerr << "CRITICAL ERROR: All methods failed to open mapping file for writing!" << std::endl;
            std::cerr << "Error: " << strerror(errno) << std::endl;
            
            // One final desperate attempt with direct file creation
            try {
                // Try to write to a test file in the same directory
                std::string test_path = abs_calibration_dir + "/test_write.txt";
                std::ofstream test_file(test_path);
                if (test_file.is_open()) {
                    test_file << "Test write" << std::endl;
                    test_file.close();
                    std::cout << "Test file created successfully, but mapping file failed." << std::endl;
                } else {
                    std::cerr << "Test file also failed. Directory may not be writable." << std::endl;
                }
            } catch (const std::exception& e) {
                std::cerr << "Exception during test write: " << e.what() << std::endl;
            }
            
            return;
        }

        // Collect mappings first
        std::vector<std::pair<std::string, std::string>> mappings;
        for (const auto& drone : drones) {
            if (!drone.tracker_id.empty()) {
                mappings.push_back({drone.mac, drone.tracker_id});
                std::cout << "CRITICAL: Found mapping: " << drone.mac 
                         << " -> " << drone.tracker_id << std::endl;
            }
        }
        
        // Now write them all
        int mapped_count = 0;
        for (const auto& mapping : mappings) {
            std::string line = mapping.first + "," + mapping.second + "\n";
            file << line;
            
            // Debug output
            std::cout << "CRITICAL: Writing line to file: " << line;
            mapped_count++;
            
            // Flush after each line to ensure it's written
            file.flush();
        }
        
        // Final flush and close
        file.flush();
        file.close();
        
        if (mapped_count > 0) {
            std::cout << "CRITICAL: Saved " << mapped_count << " tracker mappings to " << file_path << std::endl;
            
            // Verify the file was written correctly
            try {
                std::ifstream check_file(file_path);
                if (check_file.is_open()) {
                    std::string content;
                    std::string line;
                    int read_count = 0;
                    
                    while (std::getline(check_file, line)) {
                        content += line + "\n";
                        read_count++;
                        std::cout << "CRITICAL: Verification read: " << line << std::endl;
                    }
                    
                    check_file.close();
                    
                    std::cout << "CRITICAL: File content verification: " << content.length() 
                             << " bytes read, " << read_count << " lines found." << std::endl;
                    
                    if (read_count != mapped_count) {
                        std::cerr << "WARNING: Read " << read_count << " lines but wrote " 
                                 << mapped_count << " lines!" << std::endl;
                    } else {
                        std::cout << "CRITICAL: Verification successful!" << std::endl;
                    }
                } else {
                    std::cerr << "CRITICAL ERROR: Could not open file for verification!" << std::endl;
                }
            } catch (const std::exception& e) {
                std::cerr << "Exception during verification: " << e.what() << std::endl;
            }
        } else {
            std::cout << "WARNING: No mappings were saved because no drones have tracker IDs assigned" << std::endl;
        }
    }

    static void saveCalibration(const std::vector<DroneConfig>& drones) {
        // Create directory if it doesn't exist
        std::filesystem::create_directories(calibration_dir);
        
        std::ofstream file(calibration_dir + "/" + calibration_file);
        if (!file.is_open()) {
            std::cerr << "Failed to open calibration file for writing" << std::endl;
            return;
        }

        for (const auto& drone : drones) {
            file << drone.mac << ","
                 << drone.pitch_bias << ","
                 << drone.roll_bias << ","
                 << drone.yaw_bias << "\n";
            std::cout << "Saving " << drone.mac << " bias: pitch=" 
                     << drone.pitch_bias << " roll=" << drone.roll_bias
                     << " yaw=" << drone.yaw_bias << std::endl;
        }
        
        std::cout << "Saved calibration data for " << drones.size() << " drones" << std::endl;
    }
    
    // Save reference orientation frame
    static void saveReferenceFrame(const std::array<float, 4>& orientation) {
        // Create directory if it doesn't exist
        std::filesystem::create_directories(calibration_dir);
        
        std::ofstream file(calibration_dir + "/reference_frame.txt");
        if (!file.is_open()) {
            std::cerr << "Failed to open reference frame file for writing" << std::endl;
            return;
        }
        
        file << orientation[0] << " " 
             << orientation[1] << " "
             << orientation[2] << " "
             << orientation[3] << std::endl;
             
        std::cout << "Saved reference orientation frame" << std::endl;
    }
    
    // Load reference orientation frame
    static std::optional<std::array<float, 4>> loadReferenceFrame() {
        std::ifstream file(calibration_dir + "/reference_frame.txt");
        if (!file.is_open()) {
            std::cout << "No existing reference frame found" << std::endl;
            return std::nullopt;
        }
        
        std::array<float, 4> orientation;
        if (file >> orientation[0] >> orientation[1] >> orientation[2] >> orientation[3]) {
            std::cout << "Loaded reference orientation: ["
                     << orientation[0] << ", "
                     << orientation[1] << ", "
                     << orientation[2] << ", "
                     << orientation[3] << "]" << std::endl;
            return orientation;
        } else {
            std::cerr << "Failed to parse reference frame file" << std::endl;
            return std::nullopt;
        }
    }
    
    // Helper to get the full path to the mapping file
    static std::string getMappingFilePath() {
        // Use absolute path for consistency with saveMapping
        return "/home/finn/squawkblock/squawkblock/sync-drones/build/calibration/" + mapping_file;
    }
    
    // Helper to get the full path to the calibration file
    static std::string getCalibrationFilePath() {
        return calibration_dir + "/" + calibration_file;
    }
};

// Initialize static members
std::string FileHandler::calibration_dir = "calibration";
std::string FileHandler::mapping_file = "tracker_mapping.txt";
std::string FileHandler::calibration_file = "drift_calibration.txt";