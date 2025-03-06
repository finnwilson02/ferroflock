#include <iostream>
#include "../include/FileHandler.hpp"

int main(int argc, char** argv) {
    std::cout << "DJI Device Scanner" << std::endl;
    
    // Check if we should scan the network
    bool scan_network = false;
    if (argc > 1 && std::string(argv[1]) == "--scan") {
        scan_network = true;
    }
    
    // Find drones
    auto drones = FileHandler::findDrones(scan_network);
    
    // Print summary
    int online_count = 0;
    for (const auto& drone : drones) {
        if (drone.is_online) {
            online_count++;
        }
    }
    
    std::cout << "Found " << online_count << " online drones out of " << drones.size() << " total." << std::endl;
    
    return 0;
}