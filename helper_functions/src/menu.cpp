/**
 * menu.cpp
 * 
 * Purpose: Implementation of the Menu class for user interaction
 * 
 * Data Flow:
 *   Input: User commands from terminal, drone information from JSON files
 *   Output: Actions sent to TelloController and OptiTrack systems
 * 
 * This implementation handles the interactive terminal menu, drone discovery via
 * Python scripts, and coordinates communication between system components.
 */

#include "../include/menu.h"
#include "../include/logger.h"
#include "../include/map_optitrack.h"
#include <nlohmann/json.hpp>
#include <iostream>
#include <fstream>
#include <sstream>
#include <regex>
#include <cmath>
#include <filesystem>
#include <limits>
#include <cstdlib> // For getenv

// Define a consistent path for the drone JSON file
static const std::string JSON_PATH = std::string(getenv("HOME")) + "/ferroflock/dji_devices.json";

// Constructor
Menu::Menu(OptiTrack& optitrack, TelloController& tello_controller)
    : optitrack_(optitrack), tello_controller_(tello_controller), keyboard_control_(nullptr) {
    LOG_INFO("Initializing menu system");
    
    // Create calibration module
    calibration_ = new Calibration(optitrack_, tello_controller_);
    
    // Set up default menu options
    defineDefaultActions();
}

// Destructor
Menu::~Menu() {
    LOG_INFO("Menu system shutting down");
    
    // Clean up calibration module
    if (calibration_) {
        delete calibration_;
        calibration_ = nullptr;
    }
}

// Display the menu and get user choice
void Menu::display() {
    displayMainMenu();
}

// Display the main menu
void Menu::displayMainMenu() {
    // Print header
    std::cout << header_;
    
    // Print menu options
    std::vector<std::string> valid_keys;
    
    // Sort options by alphabetical keys for consistent display
    std::vector<std::pair<std::string, MenuOption>> sorted_options;
    for (const auto& [key, option] : options_) {
        sorted_options.push_back({key, option});
    }
    
    // Sort by alphabetical key
    std::sort(sorted_options.begin(), sorted_options.end(), [](const auto& a, const auto& b) {
        return a.first < b.first;
    });
    
    // Print sorted options
    for (const auto& [key, option] : sorted_options) {
        std::cout << key << ". " << option.label;
        if (!option.description.empty()) {
            std::cout << " - " << option.description;
        }
        std::cout << std::endl;
        valid_keys.push_back(key);
    }
    
    // Print footer
    std::cout << footer_;
    
    // Get user input
    std::cout << prompt_;
    std::string input = getValidInput(valid_keys);
    
    // Execute selected action
    executeOption(input);
}

// Display a list of available drones
void Menu::displayDroneList() {
    std::cout << "\nAvailable drones:\n";
    
    for (size_t i = 0; i < drones_.size(); ++i) {
        const auto& drone = drones_[i];
        
        // Check if tracker is visible in OptiTrack
        bool tracker_visible = optitrack_.isTrackerActive(drone.tracker_id);
        
        // Check if drone has a calibrated yaw offset
        std::string calibration_status = "";
        if (drone.yaw_offset != 0.0) {
            calibration_status = " - Yaw offset: " + std::to_string(drone.yaw_offset) + " degrees";
        } else {
            calibration_status = " - Not calibrated";
        }
        
        std::cout << i + 1 << ". " << drone.name << " (IP: " << drone.ip 
                  << ", Tracker: " << drone.tracker_id << ")"
                  << "\n   Network: " << (tello_controller_.pingDrone(drone.ip) ? "ONLINE" : "OFFLINE")
                  << "  |  OptiTrack: " << (tracker_visible ? "VISIBLE" : "NOT VISIBLE")
                  << calibration_status << std::endl;
    }
}

// Add a menu option
void Menu::addOption(const std::string& key, const std::string& label, 
                   std::function<void()> action, const std::string& description) {
    MenuOption option;
    option.label = label;
    option.action = action;
    option.description = description;
    options_[key] = option;
}

// Remove a menu option
void Menu::removeOption(const std::string& key) {
    options_.erase(key);
}

// Execute an action associated with a key
bool Menu::executeOption(const std::string& key) {
    auto it = options_.find(key);
    if (it != options_.end()) {
        it->second.action();
        return true;
    }
    return false;
}

// Set the prompt text
void Menu::setPrompt(const std::string& prompt) {
    prompt_ = prompt;
}

// Set the header text
void Menu::setHeader(const std::string& header) {
    header_ = header;
}

// Set the footer text
void Menu::setFooter(const std::string& footer) {
    footer_ = footer;
}

// Find drone by IP address
DroneData* Menu::findDroneByIP(const std::string& ip) {
    for (auto& drone : drones_) {
        if (drone.ip == ip) {
            return &drone;
        }
    }
    return nullptr;
}

// Load drones from a JSON file
std::vector<DroneData> Menu::loadDronesFromJSON(const std::string& filename) {
    std::vector<DroneData> drones;
    
    // Use JSON_PATH as default if no filename provided
    std::string actual_filename = filename.empty() ? JSON_PATH : filename;
    
    std::ifstream json_file(actual_filename);
    if (!json_file.is_open()) {
        LOG_ERROR("Failed to open " + actual_filename);
        return drones;
    }
    
    // Create hardcoded mappings based on the IP addresses that match the requirements
    // Bird5 (107), Bird3 (106), Bird4 (104), Bird1 (103)
    std::map<std::string, std::string> ipToOptitrackMap = {
        {"192.168.1.107", "Bird5"},
        {"192.168.1.106", "Bird3"},
        {"192.168.1.104", "Bird4"},
        {"192.168.1.103", "Bird1"}
    };
    
    // Use nlohmann/json to parse the file
    try {
        LOG_INFO("Parsing " + actual_filename + " with nlohmann/json");
        nlohmann::json j;
        json_file >> j;

        if (!j.contains("devices") || !j["devices"].is_array()) {
            LOG_ERROR("JSON does not contain a 'devices' array in " + actual_filename);
            return drones;
        }

        int drone_count = 0;
        for (const auto& device : j["devices"]) {
            DroneData drone;
            
            // Extract IP (required)
            if (device.contains("ip") && device["ip"].is_string()) {
                drone.ip = device["ip"].get<std::string>();
                // Validate IP format
                if (drone.ip.find('.') == std::string::npos) {
                    LOG_WARNING("Invalid IP: " + drone.ip + ", skipping");
                    continue;
                }
            } else {
                LOG_WARNING("Missing IP, skipping device");
                continue;
            }

            // Extract port
            if (device.contains("port") && device["port"].is_number()) {
                drone.port = device["port"].get<int>();
            } else {
                drone.port = 8889; // Default port
                LOG_WARNING("Missing port for " + drone.ip + ", using default 8889");
            }

            // Extract name
            if (device.contains("name") && device["name"].is_string()) {
                drone.name = device["name"].get<std::string>();
            } else {
                drone.name = "Drone " + std::to_string(++drone_count);
                LOG_WARNING("Missing name for " + drone.ip + ", using '" + drone.name + "'");
            }

            // Extract tracker_id
            if (device.contains("tracker_id") && device["tracker_id"].is_string() && 
                !device["tracker_id"].get<std::string>().empty()) {
                drone.tracker_id = device["tracker_id"].get<std::string>();
            } else if (ipToOptitrackMap.find(drone.ip) != ipToOptitrackMap.end()) {
                // Use hardcoded mapping as fallback
                drone.tracker_id = ipToOptitrackMap[drone.ip];
                // Also use tracker name as drone name for consistency
                drone.name = drone.tracker_id;
                LOG_WARNING("Using hardcoded tracker_id mapping for " + drone.ip + ": " + drone.tracker_id);
            } else {
                // Last resort
                drone.tracker_id = "Bird" + std::to_string(drone_count);
                LOG_WARNING("Missing tracker_id for " + drone.ip + ", using fallback: " + drone.tracker_id);
            }

            // Extract yaw_offset
            if (device.contains("yaw_offset") && device["yaw_offset"].is_number()) {
                drone.yaw_offset = device["yaw_offset"].get<double>();
            } else {
                drone.yaw_offset = 0.0;
                LOG_WARNING("Missing yaw_offset for " + drone.ip + ", using default 0.0");
            }

            LOG_INFO("Loaded drone: IP=" + drone.ip + 
                     ", Port=" + std::to_string(drone.port) +
                     ", Name=" + drone.name + 
                     ", Tracker=" + drone.tracker_id + 
                     ", YawOffset=" + std::to_string(drone.yaw_offset));
            drones.push_back(drone);
        }
    } catch (const nlohmann::json::exception& e) {
        LOG_ERROR("JSON parsing error in " + actual_filename + ": " + e.what());
    }
    
    if (drones.empty()) {
        LOG_ERROR("No drones found in JSON file: " + actual_filename);
    } else {
        LOG_INFO("Successfully loaded " + std::to_string(drones.size()) + " drones from " + actual_filename);
    }
    
    // Store the loaded drones
    drones_ = drones;
    
    return drones;
}

// Initialize the menu system
void Menu::initialize() {
    LOG_INFO("Starting menu initialization");
    
    LOG_INFO("Checking for drones in " + JSON_PATH);
    if (std::filesystem::exists(JSON_PATH)) {
        drones_ = loadDronesFromJSON(JSON_PATH);
    } else {
        // Try the old path as a fallback
        if (std::filesystem::exists("../dji_devices.json")) {
            LOG_INFO("Using fallback path: ../dji_devices.json");
            drones_ = loadDronesFromJSON("../dji_devices.json");
            // Save to the new path
            saveDronesToJSON(JSON_PATH);
        } else {
            drones_ = loadDronesFromJSON();
        }
    }
    
    // If no drones found, set up a default drone
    if (drones_.empty()) {
        LOG_WARNING("No drones found in " + JSON_PATH + ", using default drone");
        DroneData default_drone;
        default_drone.name = "Default";
        default_drone.ip = "192.168.10.1";
        default_drone.tracker_id = "Bird1";
        drones_.push_back(default_drone);
    }
    
    // Set up OptiTrack drone mapping
    optitrack_.setupDroneMapping(drones_);
    
    LOG_INFO("Menu initialization complete");
}

// Define default menu actions
void Menu::defineDefaultActions() {
    // Add main menu options
    addOption("1", "List online drones", [this]() { handleListDrones(); });
    addOption("2", "Network scan", [this]() { handleNetworkScan(); });
    addOption("3", "Map OptiTrack to drones", [this]() { handleMapOptiTrack(); });
    addOption("4", "Calibrate drone orientation", [this]() { handleCalibrateDrone(); });
    addOption("5", "Reboot all drones", [this]() { handleRebootAllDrones(); });
    addOption("6", "Set Logging Level", [this]() { handleSetLoggingLevel(); }, "Adjust verbosity of log messages");
    addOption("Q", "Quit", [this]() { handleExit(); });
    
    /*
    // Keyboard control option - commented out for future integration
    addOption("K", "Start Keyboard Control", [this]() {
        // Initialize keyboard control if it doesn't exist yet
        if (!keyboard_control_) {
            keyboard_control_ = std::make_unique<KeyboardControl>(tello_controller_);
        }
        
        // Start keyboard control
        std::cout << "Starting keyboard control mode..." << std::endl;
        keyboard_control_->start();
        
        // Wait for keyboard control to finish (this blocks until keyboard control exits)
        // Note: The keyboard control loop will run until the user presses 'q'
        std::cout << "Keyboard control mode ended. Press Enter to continue..." << std::endl;
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        std::cin.get();
        
        // Return to main menu
        display();
    }, "Control drones with keyboard input");
    */
}

// Function to reboot all drones
void Menu::rebootAllDrones(bool wait_for_reboot, bool send_land_first) {
    std::cout << "Rebooting all drones...\n";
    
    if (drones_.empty()) {
        std::cout << "No drones found to reboot.\n";
        return;
    }
    
    for (const auto& drone : drones_) {
        if (send_land_first) {
            std::cout << "Sending land command to " << drone.name << " (" << drone.ip << ")\n";
            tello_controller_.sendCommand(drone.ip, "land");
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        
        std::cout << "Rebooting " << drone.name << " (" << drone.ip << ")\n";
        tello_controller_.rebootDrone(drone.ip);
    }
    
    if (wait_for_reboot) {
        std::cout << "Waiting for drones to reboot (10 seconds)...\n";
        std::this_thread::sleep_for(std::chrono::seconds(10));
        std::cout << "All drones rebooted.\n";
    } else {
        std::cout << "Reboot commands sent to all drones.\n";
    }
}

// Get user input with validation
std::string Menu::getValidInput(const std::vector<std::string>& valid_inputs) {
    std::string input;
    bool valid = false;
    
    while (!valid) {
        std::cin >> input;
        
        // Convert to uppercase for case-insensitive comparison
        std::transform(input.begin(), input.end(), input.begin(), ::toupper);
        
        // Check if input is valid
        for (const auto& valid_input : valid_inputs) {
            std::string upper_valid = valid_input;
            std::transform(upper_valid.begin(), upper_valid.end(), upper_valid.begin(), ::toupper);
            
            if (input == upper_valid) {
                valid = true;
                input = valid_input; // Use original case
                break;
            }
        }
        
        if (!valid) {
            std::cout << "Invalid input. Please try again: ";
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        }
    }
    
    return input;
}

// Action handlers
void Menu::handleListDrones() {
    displayDroneList();
    display(); // Show main menu again
}

void Menu::handleNetworkScan() {
    std::cout << "Running network scan for Tello drones...\n";
    
    // Source the drone_venv and use sudo to run the script (scapy needs root privileges)
    std::string command = "sudo -E bash -c 'source ~/drone_venv/bin/activate && python3 ../python/get_devices.py'";
    std::cout << "Executing: " << command << "\n";
    
    int result = system(command.c_str());
    
    if (result != 0) {
        std::cout << "Error running Python script. Return code: " << result << "\n";
        display();
        return;
    }
    
    std::cout << "Network scan complete.\n";
    std::cout << "Loading updated drone information...\n";
    
    // Load the updated drone information
    drones_ = loadDronesFromJSON(JSON_PATH);
    
    // Update OptiTrack drone mapping
    optitrack_.setupDroneMapping(drones_);
    
    std::cout << "Loaded " << drones_.size() << " drones from scan.\n";
    std::cout << "Press Enter to continue...";
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    std::cin.get();
    
    display(); // Show main menu again
}

// handleMapOptiTrack is implemented elsewhere in this file

void Menu::handleCalibrateDrone() {
    // Display the list of available drones
    displayDroneList();
    
    // Check if we have any drones
    if (drones_.empty()) {
        std::cout << "No drones available for calibration.\n";
        display();
        return;
    }
    
    // Check if a calibration is already in progress
    if (calibration_->isCalibrationInProgress()) {
        std::cout << "A calibration is already in progress. Please wait for it to complete.\n";
        display();
        return;
    }
    
    // Create a vector of selectable drone indices as strings
    std::vector<std::string> valid_inputs;
    for (size_t i = 0; i < drones_.size(); ++i) {
        valid_inputs.push_back(std::to_string(i + 1));
    }
    valid_inputs.push_back("Q"); // Option to quit calibration
    
    std::cout << "\nSelect a drone to calibrate (or Q to return to main menu): ";
    std::string input = getValidInput(valid_inputs);
    
    // Check if user wants to quit
    if (input == "Q") {
        std::cout << "Calibration canceled.\n";
        display(); // Return to main menu
        return;
    }
    
    // Get the selected drone
    int drone_idx = std::stoi(input) - 1;
    DroneData& drone = drones_[drone_idx];
    
    // Start the calibration process
    std::cout << "\nStarting calibration for " << drone.name << " (" << drone.ip << ")\n";
    std::cout << "Please ensure the drone is placed on a flat surface.\n";
    std::cout << "Press Enter when ready (or Q to cancel)...";
    
    // Clear input buffer
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    
    // Check for cancel
    std::string ready;
    std::getline(std::cin, ready);
    if (ready == "Q" || ready == "q") {
        std::cout << "Calibration canceled.\n";
        display(); // Return to main menu
        return;
    }
    
    // Start the calibration routine
    calibration_->calibrateDroneOrientation(drone, drone_idx, [this]() {
        // This callback will be called when the calibration is complete
        display(); // Return to main menu
    });
}

void Menu::handleMapOptiTrack() {
    std::cout << "Starting OptiTrack to drone mapping process...\n";
    OptiTrackMapper mapper(optitrack_, tello_controller_, drones_);
    auto mapping = mapper.startMapping();
    if (mapping) {
        auto [ip, tracker_id] = *mapping;
        for (auto& drone : drones_) {
            if (drone.ip == ip) {
                drone.tracker_id = tracker_id;
                break;
            }
        }
        saveDronesToJSON(JSON_PATH);
        std::cout << "Mapping " << tracker_id << " to " << ip << " saved.\n";
    } else {
        std::cout << "Mapping canceled or failed.\n";
    }
    display();
}

void Menu::handleRebootAllDrones() {
    rebootAllDrones(true, false);
    display(); // Show main menu again
}

void Menu::handleSetLoggingLevel() {
    std::cout << "\nCurrent Logging Level: ";
    switch (g_log_level) {
        case LOG_LEVEL_DEBUG: std::cout << "DEBUG"; break;
        case LOG_LEVEL_INFO: std::cout << "INFO"; break;
        case LOG_LEVEL_WARNING: std::cout << "WARNING"; break;
        case LOG_LEVEL_ERROR: std::cout << "ERROR"; break;
        case LOG_LEVEL_NONE: std::cout << "NONE"; break;
    }
    std::cout << "\nSelect new logging level:\n";
    std::cout << "1. DEBUG - Show all messages\n";
    std::cout << "2. INFO - Show info, warnings, and errors\n";
    std::cout << "3. WARNING - Show warnings and errors\n";
    std::cout << "4. ERROR - Show only errors\n";
    std::cout << "5. NONE - Suppress all messages\n";
    std::cout << "Q. Cancel\n";
    std::cout << "Enter choice: ";

    std::vector<std::string> valid_inputs = {"1", "2", "3", "4", "5", "Q"};
    std::string input = getValidInput(valid_inputs);

    if (input == "Q") {
        std::cout << "Logging level unchanged.\n";
    } else {
        if (input == "1") g_log_level = LOG_LEVEL_DEBUG;
        else if (input == "2") g_log_level = LOG_LEVEL_INFO;
        else if (input == "3") g_log_level = LOG_LEVEL_WARNING;
        else if (input == "4") g_log_level = LOG_LEVEL_ERROR;
        else if (input == "5") g_log_level = LOG_LEVEL_NONE;
        std::cout << "Logging level set to " << input << ".\n";
    }
    display();
}

void Menu::handleExit() {
    std::cout << "Exiting program...\n";
    
    // Perform cleanup before exit
    LOG_INFO("Performing cleanup before exit");
    rebootAllDrones(false, true);
    
    // Exit the program
    exit(0);
}

void Menu::saveDronesToJSON(const std::string& filename) {
    nlohmann::json j_existing;
    
    // Try to read existing file if it exists
    if (std::filesystem::exists(filename)) {
        std::ifstream json_file(filename);
        if (json_file.is_open()) {
            try {
                json_file >> j_existing;
                LOG_INFO("Merging with existing JSON file: " + filename);
            } catch (const nlohmann::json::exception& e) {
                LOG_WARNING("Failed to parse existing JSON " + filename + ": " + e.what() + ". Creating new file.");
            }
            json_file.close();
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
        
        // If device already exists, start with its existing data
        if (existing_devices.find(drone.ip) != existing_devices.end()) {
            device = existing_devices[drone.ip];
        }
        
        // Update with current values
        device["ip"] = drone.ip;
        device["name"] = drone.name;
        device["tracker_id"] = drone.tracker_id;
        device["yaw_offset"] = drone.yaw_offset;

        // Store in our map
        existing_devices[drone.ip] = device;
    }

    // Rebuild the devices array with updated data
    j_existing["devices"] = nlohmann::json::array();
    for (const auto& [ip, device] : existing_devices) {
        j_existing["devices"].push_back(device);
    }

    // Write the updated JSON to file
    std::ofstream file(filename);
    if (file.is_open()) {
        file << j_existing.dump(4); // Pretty print with 4-space indentation
        file.close();
        LOG_INFO("Saved " + std::to_string(j_existing["devices"].size()) + " drones to " + filename);
    } else {
        LOG_ERROR("Failed to save drones to " + filename);
        std::cout << "Failed to save mappings.\n";
    }
}