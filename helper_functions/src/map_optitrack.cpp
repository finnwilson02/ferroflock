/**
 * map_optitrack.cpp
 * 
 * Purpose: Implementation of OptiTrackMapper for drone-tracker association
 * 
 * Data Flow:
 *   Input: User selections, drone movement via keyboard control
 *   Output: Drone-to-tracker mappings in JSON configuration
 * 
 * This implementation handles the process of mapping OptiTrack trackers
 * to Tello drones by flying a selected drone and detecting which tracker
 * has the most movement.
 */

#include "../include/map_optitrack.h"
#include "../include/logger.h"
#include <iostream>
#include <fstream>
#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <filesystem> // For directory creation
#include <nlohmann/json.hpp>
#include <cstdlib> // For getenv
#include <termios.h> // For terminal control
#include <unistd.h> // For STDIN_FILENO
#include <cstdio>  // For scanf and getchar
#include <fcntl.h> // For fcntl

// Define a consistent path for the drone JSON file
static const std::string JSON_PATH = std::string(getenv("HOME")) + "/ferroflock/dji_devices.json";

// Constructor
OptiTrackMapper::OptiTrackMapper(OptiTrack& optitrack, TelloController& controller, 
                               std::vector<DroneData>& drones)
    : optitrack_(optitrack), controller_(controller), drones_(drones) {
    LOG_INFO("OptiTrackMapper initialized");
}

// Helper function to get valid input from user
std::string OptiTrackMapper::getValidInput(const std::vector<std::string>& valid_inputs) {
    while (true) {
        std::cout << "Enter your choice (Y/N): ";
        std::cout.flush(); // Ensure prompt displays immediately
        std::string input;
        
        // Set terminal to blocking mode for reliable input
        int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
        fcntl(STDIN_FILENO, F_SETFL, flags & ~O_NONBLOCK);
        
        // Try to read a line
        try {
            std::getline(std::cin, input);
            LOG_DEBUG("getline read: '" + input + "'");
        }
        catch (const std::exception& e) {
            LOG_DEBUG("Exception during getline: " + std::string(e.what()));
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            std::cout << "Input error, please try again." << std::endl;
            continue;
        }

        // Check for input stream failure
        if (std::cin.fail()) {
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            std::cout << "Input error, please try again." << std::endl;
            LOG_DEBUG("std::cin.fail() detected, cleared state");
            continue;
        }

        // Trim leading and trailing whitespace
        input.erase(0, input.find_first_not_of(" \t\n\r\f\v"));
        input.erase(input.find_last_not_of(" \t\n\r\f\v") + 1);

        if (input.empty()) {
            std::cout << "Invalid input. Please try again." << std::endl;
            LOG_DEBUG("Empty input received");
            continue;
        }

        // Case-insensitive comparison
        std::string upper_input = input;
        std::transform(upper_input.begin(), upper_input.end(), upper_input.begin(), ::toupper);

        for (const auto& valid : valid_inputs) {
            std::string upper_valid = valid;
            std::transform(upper_valid.begin(), upper_valid.end(), upper_valid.begin(), ::toupper);
            if (upper_input == upper_valid) {
                LOG_DEBUG("Valid input received: '" + input + "'");
                return input;
            }
        }

        std::cout << "Invalid input. Please try again." << std::endl;
        LOG_DEBUG("Invalid input: '" + input + "'");
    }
}

// Display status of OptiTrack trackers
void OptiTrackMapper::displayTrackerStatus() {
    // Create a map of tracker IDs to drone IPs
    std::map<std::string, std::string> tracker_to_ip;
    for (const auto& drone : drones_) {
        if (!drone.tracker_id.empty()) {
            tracker_to_ip[drone.tracker_id] = drone.ip;
        }
    }
    
    // Get list of active trackers
    auto active_trackers = optitrack_.getActiveTrackers();
    
    std::cout << "\nOptiTrack is broadcasting the following trackers:\n";
    for (const auto& tracker : active_trackers) {
        auto it = tracker_to_ip.find(tracker);
        if (it != tracker_to_ip.end()) {
            std::cout << "  " << tracker << " (MAPPED: " << it->second << ")\n";
        } else {
            std::cout << "  " << tracker << " (UNMAPPED)\n";
        }
    }
    
    if (active_trackers.empty()) {
        std::cout << "  No active trackers detected. Please check OptiTrack system.\n";
    }
}

// Display status of online drones
void OptiTrackMapper::displayDroneStatus() {
    // Find online drones
    std::vector<std::pair<int, DroneData>> online_drones;
    for (size_t i = 0; i < drones_.size(); ++i) {
        if (controller_.pingDrone(drones_[i].ip)) {
            online_drones.push_back({i, drones_[i]});
        }
    }
    
    std::cout << "\nAvailable online drones:\n";
    for (size_t i = 0; i < online_drones.size(); ++i) {
        const auto& [idx, drone] = online_drones[i];
        std::string mapping_status = drone.tracker_id.empty() ? 
                                    "Unmapped" : 
                                    ("Mapped to " + drone.tracker_id);
        std::cout << (i + 1) << ". " << drone.name << " (IP: " << drone.ip 
                  << ") - " << mapping_status << "\n";
    }
    
    if (online_drones.empty()) {
        std::cout << "  No online drones detected. Please check drone power and network.\n";
    }
    
    return;
}

// Main mapping function
std::optional<std::pair<std::string, std::string>> OptiTrackMapper::startMapping() {
    // Display OptiTrack tracker status
    displayTrackerStatus();
    
    // Display drone status
    displayDroneStatus();
    
    // Find online drones
    std::vector<std::pair<int, DroneData>> online_drones;
    for (size_t i = 0; i < drones_.size(); ++i) {
        if (controller_.pingDrone(drones_[i].ip)) {
            online_drones.push_back({i, drones_[i]});
        }
    }
    
    if (online_drones.empty()) {
        std::cout << "No online drones detected. Mapping cannot proceed.\n";
        return std::nullopt;
    }
    
    // Allow user to select a drone
    std::vector<std::string> valid_inputs;
    for (size_t i = 0; i < online_drones.size(); ++i) {
        valid_inputs.push_back(std::to_string(i + 1));
    }
    valid_inputs.push_back("Q");
    
    std::cout << "\nSelect a drone to map (or Q to return to main menu): ";
    std::string input = getValidInput(valid_inputs);
    
    // Check if user wants to quit
    if (input == "Q" || input == "q") {
        std::cout << "Mapping canceled.\n";
        return std::nullopt;
    }
    
    // Get selected drone
    int selection = std::stoi(input) - 1;
    auto [drone_idx, selected_drone] = online_drones[selection];
    
    // Print instructions
    std::cout << "\nMapping drone " << selected_drone.name << " (IP: " << selected_drone.ip << ")\n";
    std::cout << "Please fly the drone for at least 10 seconds to ensure accurate mapping.\n";
    std::cout << "Controls:\n";
    std::cout << "  't' - takeoff\n";
    std::cout << "  'l' - land\n";
    std::cout << "  'w/s' - forward/backward\n";
    std::cout << "  'a/d' - left/right\n";
    std::cout << "  'arrow keys' - up/down/rotate\n";
    std::cout << "  'q' - finish flying and detect tracker\n";
    std::cout << "Press any key to begin...\n";
    
    // Wait for user to press a key
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    std::cin.get();
    
    // Initialize movement tracking
    std::map<std::string, std::vector<Position>> tracker_positions;
    
    // Start keyboard control for the selected drone
    KeyboardControl keyboard_control(controller_, selected_drone.ip);
    keyboard_control.start();
    
    std::cout << "Keyboard control started. Flying drone " << selected_drone.name << "...\n";
    
    // Collect tracker positions while the drone is flying
    while (keyboard_control.isRunning()) {
        auto now = std::chrono::system_clock::now();
        auto active_trackers = optitrack_.getActiveTrackers();
        for (const auto& tracker : active_trackers) {
            if (optitrack_.isTrackerActive(tracker)) {
                Position pos;
                pos.x = optitrack_.getXPosition(tracker);
                pos.y = optitrack_.getYPosition(tracker);
                pos.z = optitrack_.getZPosition(tracker);
                pos.timestamp = now;
                tracker_positions[tracker].push_back(pos);
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    // Ensure keyboard control thread has stopped
    keyboard_control.stop();
    
    // Add a brief delay to ensure terminal settings stabilize
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // Fully reset terminal to canonical mode
    struct termios term;
    tcgetattr(STDIN_FILENO, &term);
    term.c_lflag |= (ICANON | ECHO); // Enable canonical mode and echo
    term.c_cc[VMIN] = 1;             // Wait for at least 1 character
    term.c_cc[VTIME] = 0;            // No timeout
    tcsetattr(STDIN_FILENO, TCSAFLUSH, &term);
    LOG_DEBUG("Terminal fully reset after keyboard control (VMIN=1, VTIME=0)");
    
    // Fully clear the input buffer
    std::cin.clear(); // Clear any error flags
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    
    // Also use getchar to ensure no characters remain
    int c;
    fcntl(STDIN_FILENO, F_SETFL, 0); // Set blocking mode
    while ((c = getchar()) != EOF && c != '\n'); // Flush residual characters
    
    // Reset file descriptor flags back to normal
    fcntl(STDIN_FILENO, F_SETFL, 0);
    
    // Clear any error states
    clearerr(stdin);
    LOG_DEBUG("Input buffer fully cleared after keyboard control");
    
    // Calculate total movement distance for each tracker
    std::map<std::string, double> total_distances;
    for (const auto& [tracker, positions] : tracker_positions) {
        double distance = 0.0;
        for (size_t i = 1; i < positions.size(); ++i) {
            double dx = positions[i].x - positions[i-1].x;
            double dy = positions[i].y - positions[i-1].y;
            double dz = positions[i].z - positions[i-1].z;
            distance += std::sqrt(dx*dx + dy*dy + dz*dz);
        }
        total_distances[tracker] = distance;
        LOG_DEBUG("Tracker " + tracker + " moved " + std::to_string(distance) + " meters");
    }
    
    // Find the tracker with the most movement
    if (total_distances.empty()) {
        std::cout << "No tracker data collected. Please try again with longer flight time.\n";
        return std::nullopt;
    }
    
    // Find maximum movement tracker
    auto max_it = std::max_element(
        total_distances.begin(), total_distances.end(),
        [](const auto& a, const auto& b) { return a.second < b.second; }
    );
    
    std::string moving_tracker = max_it->first;
    double max_distance = max_it->second;
    
    // Check if movement was significant
    if (max_distance <= 0.5) {
        std::cout << "No significant movement detected. Please try again with more movement.\n";
        return std::nullopt;
    }
    
    // Print movement results
    std::cout << "\nTracker movement analysis:\n";
    for (const auto& [tracker, distance] : total_distances) {
        std::cout << "  " << tracker << ": " << distance << " meters";
        if (tracker == moving_tracker) {
            std::cout << " (MAXIMUM)";
        }
        std::cout << "\n";
    }
    
    std::cout << "\nDetected " << moving_tracker << " as the most active tracker "
              << "(" << max_distance << " meters moved).\n";
    
    // Check if this tracker is already mapped to another drone
    auto existing_it = std::find_if(drones_.begin(), drones_.end(),
        [&moving_tracker](const DroneData& d) { return d.tracker_id == moving_tracker; }
    );
    
    std::string overwrite_warning = "";
    if (existing_it != drones_.end() && existing_it->ip != selected_drone.ip) {
        overwrite_warning = " (Note: This will overwrite the existing mapping to " 
                           + existing_it->ip + ")";
    }
    
    // Confirm mapping with user
    std::cout << "Do you want to map " << moving_tracker << " to drone " 
              << selected_drone.name << " (IP: " << selected_drone.ip << ")"
              << overwrite_warning << "? ";
    valid_inputs = {"Y", "N"};
    LOG_DEBUG("Waiting for Y/N confirmation");
    std::cout.flush(); // Ensure prompt is visible
    
    // Ensure stdin is in blocking mode
    int orig_flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, orig_flags & ~O_NONBLOCK);
    
    // Clear cin before getting input
    std::cin.clear();
    std::cin.sync();
    
    // Get input from user
    input = getValidInput(valid_inputs);
    LOG_DEBUG("Received confirmation input: '" + input + "'");

    if (input == "Y") {
        LOG_DEBUG("User confirmed mapping: " + moving_tracker + " to " + selected_drone.ip);
        drones_[drone_idx].tracker_id = moving_tracker;
        std::cout << "Mapping updated: " << moving_tracker << " -> " 
                  << selected_drone.name << " (IP: " << selected_drone.ip << ")\n";
        
        LOG_INFO("Attempting to save mapping to " + std::string(JSON_PATH));
        bool save_result = saveDronesToJSON(JSON_PATH);
        LOG_DEBUG("saveDronesToJSON returned: " + std::string(save_result ? "true" : "false"));
        
        if (save_result) {
            std::cout << "Mapping successfully saved to " << JSON_PATH << "\n";
            LOG_INFO("Mapping successfully saved");
        } else {
            std::cout << "Failed to save mapping to " << JSON_PATH << "\n";
            LOG_ERROR("Failed to save mapping");
        }
        
        return std::make_pair(selected_drone.ip, moving_tracker);
    } else {
        std::cout << "Mapping canceled.\n";
        LOG_INFO("Mapping canceled by user");
        return std::nullopt;
    }
}

// Save drone data to JSON file
bool OptiTrackMapper::saveDronesToJSON(const std::string& filename) {
    // Use JSON_PATH as default if no filename provided
    std::string actual_filename = filename.empty() ? JSON_PATH : filename;
    nlohmann::json j_existing;

    // Ensure the directory exists
    std::string dir_path = "/home/finn/ferroflock";
    if (!std::filesystem::exists(dir_path)) {
        try {
            std::filesystem::create_directories(dir_path);
            LOG_INFO("Created directory: " + dir_path);
        } catch (const std::exception& e) {
            LOG_ERROR("Failed to create directory " + dir_path + ": " + std::string(e.what()));
            return false;
        }
    }

    // Try to read existing file if it exists
    if (std::filesystem::exists(actual_filename)) {
        std::ifstream json_file(actual_filename);
        if (json_file.is_open()) {
            try {
                json_file >> j_existing;
                LOG_INFO("Merging with existing JSON file: " + actual_filename);
            } catch (const nlohmann::json::exception& e) {
                LOG_WARNING("Failed to parse existing JSON " + actual_filename + ": " + e.what() + ". Creating new file.");
            }
            json_file.close();
        } else {
            LOG_WARNING("Could not open existing JSON file for reading: " + actual_filename);
        }
    }

    // Initialize devices array if it doesn't exist
    if (!j_existing.contains("devices") || !j_existing["devices"].is_array()) {
        j_existing["devices"] = nlohmann::json::array();
    }

    // Create a map of existing devices by IP for quick lookup
    std::map<std::string, nlohmann::json> existing_devices;
    for (const auto& device : j_existing["devices"]) {
        if (device.contains("ip") && device["ip"].is_string()) {
            existing_devices[device["ip"].get<std::string>()] = device;
        }
    }

    // Update or add drones from drones_
    for (const auto& drone : drones_) {
        nlohmann::json device;
        if (existing_devices.find(drone.ip) != existing_devices.end()) {
            device = existing_devices[drone.ip];
        }
        device["ip"] = drone.ip;
        device["name"] = drone.name;
        device["tracker_id"] = drone.tracker_id;
        device["yaw_offset"] = drone.yaw_offset;
        existing_devices[drone.ip] = device;
    }

    // Rebuild the devices array
    j_existing["devices"] = nlohmann::json::array();
    for (const auto& [ip, device] : existing_devices) {
        j_existing["devices"].push_back(device);
    }

    // Write the updated JSON to file
    try {
        std::ofstream file(actual_filename);
        if (file.is_open()) {
            file << j_existing.dump(4); // Pretty print with 4-space indentation
            file.close();
            LOG_INFO("Saved " + std::to_string(j_existing["devices"].size()) + " drones to " + actual_filename);
            return true;
        } else {
            LOG_ERROR("Failed to open file for writing: " + actual_filename + " (check permissions or file lock)");
            return false;
        }
    } catch (const std::exception& e) {
        LOG_ERROR("Exception when saving JSON: " + std::string(e.what()));
        return false;
    }
}