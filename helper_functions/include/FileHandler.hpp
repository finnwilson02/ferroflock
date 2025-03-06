// include/FileHandler.hpp
#pragma once
#include <string>
#include <vector>
#include <fstream>
#include <iostream>
#include <filesystem>
#include <sstream>
#include <regex>
#include <cstdlib>

struct DroneConfig {
    std::string mac;
    std::string ip;
    bool is_online{false};    // Flag to indicate if drone was found in network scan
    std::string last_seen;    // Timestamp when drone was last seen
};

class FileHandler {
private:
    static std::string calibration_dir;

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
            std::string python_script = "/home/finn/squawkblock/squawkblock/frame-calibration/python/get_devices.py";
            
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
};

// Initialize static member
std::string FileHandler::calibration_dir = "calibration";