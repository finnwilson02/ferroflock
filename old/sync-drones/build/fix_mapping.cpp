#include <iostream>
#include <fstream>
#include <string>
#include <cstring>
#include <cerrno>
#include <filesystem>

// Function to create a tracker mapping file
bool writeTrackerMapping(const std::string& mac, const std::string& tracker_id) {
    // Use absolute path for reliable file location
    std::string file_path = "/home/finn/squawkblock/squawkblock/sync-drones/build/calibration/tracker_mapping.txt";
    
    // Create directory if it doesn't exist
    std::filesystem::create_directories("/home/finn/squawkblock/squawkblock/sync-drones/build/calibration");
    
    // Open file for writing (create or truncate)
    std::ofstream file(file_path);
    if (!file.is_open()) {
        std::cerr << "Failed to open tracker mapping file: " << strerror(errno) << std::endl;
        return false;
    }
    
    // Write the mapping
    file << mac << "," << tracker_id << std::endl;
    
    // Ensure data is written
    file.flush();
    
    // Check for errors
    if (file.fail()) {
        std::cerr << "Error writing to file: " << strerror(errno) << std::endl;
        file.close();
        return false;
    }
    
    // Close the file
    file.close();
    
    // Verify the file was written
    std::ifstream verify(file_path);
    if (!verify.is_open()) {
        std::cerr << "Failed to open file for verification: " << strerror(errno) << std::endl;
        return false;
    }
    
    std::string line;
    bool found_mapping = false;
    while (std::getline(verify, line)) {
        std::cout << "Read line: " << line << std::endl;
        if (line.find(mac) != std::string::npos && line.find(tracker_id) != std::string::npos) {
            found_mapping = true;
        }
    }
    
    verify.close();
    
    if (!found_mapping) {
        std::cerr << "Verification failed: Mapping not found in file" << std::endl;
        return false;
    }
    
    std::cout << "Successfully wrote and verified mapping" << std::endl;
    return true;
}

int main() {
    std::cout << "This program will create a tracker mapping file with a test mapping" << std::endl;
    
    // Replace with a real MAC and tracker ID
    std::string mac = "34:D2:62:XX:XX:XX";  // Example MAC
    std::string tracker_id = "vicon_drone1";    // Example tracker ID
    
    std::cout << "Writing mapping: " << mac << " -> " << tracker_id << std::endl;
    
    if (writeTrackerMapping(mac, tracker_id)) {
        std::cout << "SUCCESS: Mapping file created and verified" << std::endl;
    } else {
        std::cerr << "FAILURE: Could not create mapping file" << std::endl;
    }
    
    return 0;
}