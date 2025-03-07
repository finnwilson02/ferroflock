#include <iostream>
#include <fstream>
#include <string>
#include <chrono>
#include <thread>
#include <cstring>
#include <cerrno>
#include <vector>
#include <random>

// Function to log position data to a file
bool logPositions(const std::string& ip) {
    // Use absolute path for reliable file location
    std::string file_path = "/home/finn/squawkblock/squawkblock/sync-drones/build/positions_" + ip + ".txt";
    
    // Open file for writing (create or truncate)
    std::ofstream file(file_path);
    if (!file.is_open()) {
        std::cerr << "Failed to open position log file: " << strerror(errno) << std::endl;
        return false;
    }
    
    // Write header
    file << "timestamp,tracker_id,x,y,z" << std::endl;
    file.flush();
    
    // Setup random number generator for simulated position data
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(-1.0, 1.0);
    
    // Define some tracker IDs
    std::vector<std::string> trackers = {"vicon_drone1", "vicon_drone2", "vicon_marker3"};
    
    // Simulate logging position data for a few seconds
    std::cout << "Logging positions for 3 seconds..." << std::endl;
    
    for (int i = 0; i < 10; i++) {
        // Get current timestamp
        auto now = std::chrono::system_clock::now();
        auto timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
                      now.time_since_epoch()).count();
        
        // Log position for each tracker
        for (const auto& tracker : trackers) {
            // Generate random position data
            double x = dis(gen);
            double y = dis(gen);
            double z = dis(gen);
            
            // Write position data
            file << timestamp << "," << tracker << "," 
                 << x << "," << y << "," << z << std::endl;
            
            std::cout << "Logged position for " << tracker << ": "
                      << x << ", " << y << ", " << z << std::endl;
        }
        
        // Force flush after each set of writes
        file.flush();
        
        // Sleep for a moment
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
    }
    
    // Close the file
    file.close();
    
    // Verify the file was written
    std::ifstream verify(file_path);
    if (!verify.is_open()) {
        std::cerr << "Failed to open file for verification: " << strerror(errno) << std::endl;
        return false;
    }
    
    // Count lines in the file
    int line_count = 0;
    std::string line;
    while (std::getline(verify, line)) {
        line_count++;
    }
    
    verify.close();
    
    // We should have header + 10 samples × 3 trackers = 31 lines
    std::cout << "File contains " << line_count << " lines (expected 31)" << std::endl;
    
    if (line_count < 31) {
        std::cerr << "Verification failed: Not all data was written to file" << std::endl;
        return false;
    }
    
    std::cout << "Successfully wrote and verified position log" << std::endl;
    return true;
}

int main() {
    std::cout << "This program will create a position log file with simulated data" << std::endl;
    
    // Use IP address of a real drone
    std::string ip = "192.168.1.104";
    
    std::cout << "Creating position log for drone at IP: " << ip << std::endl;
    
    if (logPositions(ip)) {
        std::cout << "SUCCESS: Position log created and verified" << std::endl;
    } else {
        std::cerr << "FAILURE: Could not create position log" << std::endl;
    }
    
    return 0;
}