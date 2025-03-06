#include <iostream>
#include <fstream>
#include <string>
#include <cstring>
#include <cerrno>
#include <filesystem>
#include <vector>
#include <chrono>
#include <map>
#include <array>

// Simple utility functions for reliable file writing

namespace FileWriter {
    
    // Save drone-to-tracker mapping
    bool saveTrackerMapping(const std::string& mac, const std::string& trackerId) {
        // Use absolute path for reliability
        std::string dirPath = "/home/finn/squawkblock/squawkblock/sync-drones/build/calibration";
        std::string filePath = dirPath + "/tracker_mapping.txt";
        
        // Create directory if it doesn't exist
        try {
            std::filesystem::create_directories(dirPath);
        } catch (const std::exception& e) {
            std::cerr << "Error creating directory: " << e.what() << std::endl;
        }
        
        // Open file for writing (append mode)
        std::ofstream file(filePath, std::ios::out | std::ios::app);
        if (!file.is_open()) {
            std::cerr << "Failed to open mapping file: " << strerror(errno) << std::endl;
            return false;
        }
        
        // Write the mapping
        file << mac << "," << trackerId << std::endl;
        file.flush();
        
        if (file.fail()) {
            std::cerr << "Error writing to mapping file: " << strerror(errno) << std::endl;
            file.close();
            return false;
        }
        
        file.close();
        
        // Verify the file was written
        std::ifstream verify(filePath);
        if (!verify.is_open()) {
            std::cerr << "Failed to verify mapping file: " << strerror(errno) << std::endl;
            return false;
        }
        
        std::string line;
        bool found = false;
        while (std::getline(verify, line)) {
            if (line.find(mac) != std::string::npos && line.find(trackerId) != std::string::npos) {
                found = true;
                break;
            }
        }
        verify.close();
        
        if (!found) {
            std::cerr << "Mapping verification failed - entry not found in file" << std::endl;
            return false;
        }
        
        std::cout << "Mapping saved and verified: " << mac << " -> " << trackerId << std::endl;
        return true;
    }
    
    // Create a position log file
    std::ofstream createPositionLog(const std::string& ip) {
        // Use absolute path for reliability
        std::string filePath = "/home/finn/squawkblock/squawkblock/sync-drones/build/positions_" + ip + ".txt";
        
        // Open file for writing (truncate any existing content)
        std::ofstream file(filePath, std::ios::out | std::ios::trunc);
        if (!file.is_open()) {
            std::cerr << "Failed to create position log: " << strerror(errno) << std::endl;
            return std::ofstream(); // Return invalid stream
        }
        
        // Write header
        file << "timestamp,tracker_id,x,y,z" << std::endl;
        file.flush();
        
        if (file.fail()) {
            std::cerr << "Error writing header to position log: " << strerror(errno) << std::endl;
            file.close();
            return std::ofstream(); // Return invalid stream
        }
        
        std::cout << "Position log created at " << filePath << std::endl;
        return file; // Return the valid file stream
    }
    
    // Log a position entry
    bool logPosition(std::ofstream& file, const std::string& trackerId, 
                     float x, float y, float z) {
        if (!file.is_open()) {
            std::cerr << "Position log file is not open" << std::endl;
            return false;
        }
        
        // Get current timestamp
        auto now = std::chrono::system_clock::now();
        auto timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
                        now.time_since_epoch()).count();
        
        // Write position data
        file << timestamp << "," << trackerId << "," 
             << x << "," << y << "," << z << std::endl;
        file.flush();
        
        if (file.fail()) {
            std::cerr << "Error writing to position log: " << strerror(errno) << std::endl;
            return false;
        }
        
        return true;
    }
}

// This file doesn't have a main function - it's meant to be included in the main program