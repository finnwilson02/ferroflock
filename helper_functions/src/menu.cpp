#include "../include/menu.h"
#include "../include/logger.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <regex>
#include <cmath>
#include <filesystem>
#include <limits>

// Constructor
Menu::Menu(OptiTrack& optitrack, TelloController& tello_controller)
    : optitrack_(optitrack), tello_controller_(tello_controller) {
    LOG_INFO("Initializing menu system");
    
    // Set up default menu options
    defineDefaultActions();
}

// Destructor
Menu::~Menu() {
    LOG_INFO("Menu system shutting down");
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
    std::ifstream json_file(filename);
    
    if (!json_file.is_open()) {
        LOG_ERROR("Failed to open " + filename);
        return drones;
    }
    
    // Create hardcoded mappings based on the IP addresses that match the requirements
    // Bird5 (107), Bird3 (106), Bird4 (104), Bird1 (108)
    std::map<std::string, std::string> ipToOptitrackMap = {
        {"192.168.1.107", "Bird5"},
        {"192.168.1.106", "Bird3"},
        {"192.168.1.104", "Bird4"},
        {"192.168.1.108", "Bird1"}
    };
    
    // Read the entire file into a string
    std::stringstream buffer;
    buffer << json_file.rdbuf();
    std::string content = buffer.str();
    
    // Just parse IP addresses from the file
    std::regex ip_pattern("\"ip\"\\s*:\\s*\"([^\"]+)\"");
    
    auto ips_begin = std::sregex_iterator(content.begin(), content.end(), ip_pattern);
    auto ips_end = std::sregex_iterator();
    
    int drone_count = 0;
    for (std::sregex_iterator i = ips_begin; i != ips_end; ++i) {
        std::smatch match = *i;
        if (match.size() > 1) {
            std::string ip = match[1].str();
            DroneData drone;
            drone.ip = ip;
            drone.name = "Drone " + std::to_string(++drone_count);
            
            // Assign OptiTrack name based on IP
            if (ipToOptitrackMap.find(ip) != ipToOptitrackMap.end()) {
                drone.tracker_id = ipToOptitrackMap[ip];
                drone.name = drone.tracker_id;
            } else {
                drone.tracker_id = "Bird" + std::to_string(drone_count);
            }
            
            LOG_INFO("Loaded drone: IP=" + drone.ip + 
                   ", Name=" + drone.name +
                   ", OptiTrack Name=" + drone.tracker_id);
            drones.push_back(drone);
        }
    }
    
    if (drones.empty()) {
        LOG_ERROR("No drones found in JSON");
    }
    
    // Store the loaded drones
    drones_ = drones;
    
    return drones;
}

// Initialize the menu system
void Menu::initialize() {
    LOG_INFO("Starting menu initialization");
    
    // Load drones from JSON file
    LOG_INFO("Loading drones from dji_devices.json");
    drones_ = loadDronesFromJSON("dji_devices.json");
    
    // If no drones found, set up a default drone
    if (drones_.empty()) {
        LOG_WARNING("No drones found in dji_devices.json, using default drone");
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
    addOption("Q", "Quit", [this]() { handleExit(); });
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
    std::cout << "Network scan is not implemented yet.\n";
    display(); // Show main menu again
}

void Menu::handleMapOptiTrack() {
    std::cout << "Manual OptiTrack mapping is not implemented yet.\n";
    display(); // Show main menu again
}

void Menu::handleCalibrateDrone() {
    // Display the list of available drones
    displayDroneList();
    
    // Check if we have any drones
    if (drones_.empty()) {
        std::cout << "No drones available for calibration.\n";
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
    
    // Check if the drone is online
    if (!tello_controller_.pingDrone(drone.ip)) {
        std::cout << "Error: Drone " << drone.name << " is not online. Cannot calibrate.\n";
        std::this_thread::sleep_for(std::chrono::seconds(2));
        display(); // Return to main menu
        return;
    }
    
    // Initialize drone for SDK mode
    if (!tello_controller_.initialize(drone.ip, false)) {
        std::cout << "Error: Failed to initialize drone at " << drone.ip << ".\n";
        std::this_thread::sleep_for(std::chrono::seconds(2));
        display(); // Return to main menu
        return;
    }
    
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
    
    // Make sure data directory exists
    Logger::createDataDirectory("/home/finn/ferroflock/helper_functions/data");
    
    // Create a logger to record calibration data
    std::string log_filename = "/home/finn/ferroflock/helper_functions/data/" + 
                               Logger::createUniqueFilename("application_log", ".csv");
    Logger calibration_logger(log_filename);
    
    if (!calibration_logger.isOpen()) {
        std::cout << "Error: Failed to open log file. Aborting calibration.\n";
        std::this_thread::sleep_for(std::chrono::seconds(2));
        display(); // Return to main menu
        return;
    }
    
    std::cout << "Starting calibration routine...\n";
    
    // Create a detached thread to run the calibration sequence
    std::thread([this, drone_ip = drone.ip, tracker_id = drone.tracker_id, 
                name = drone.name, log_filename, drone_idx, &calibration_logger]() {
        try {
            // 1. Takeoff
            std::cout << "Step 1: Taking off...\n";
            tello_controller_.sendCommand(drone_ip, "command");
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            tello_controller_.sendCommand(drone_ip, "takeoff");
            
            // Wait for takeoff to complete
            std::this_thread::sleep_for(std::chrono::seconds(5));
            
            // Log initial position
            DataPoint takeoff_data(tracker_id);
            takeoff_data.x = optitrack_.getXPosition(tracker_id);
            takeoff_data.y = optitrack_.getYPosition(tracker_id);
            takeoff_data.z = optitrack_.getZPosition(tracker_id);
            takeoff_data.yaw_raw = optitrack_.getRawYaw(tracker_id);
            takeoff_data.yaw_corrected = optitrack_.getYaw(tracker_id);
            takeoff_data.commanded_yaw = 0.0; // No command yet
            calibration_logger.logData(takeoff_data);
            
            // 2. Move upward for 3 seconds
            std::cout << "Step 2: Moving upward for 3 seconds...\n";
            tello_controller_.sendCommand(drone_ip, "up 50");
            
            // Log positions while moving upward (30 samples over 3 seconds)
            for (int i = 0; i < 30; i++) {
                DataPoint data(tracker_id);
                data.x = optitrack_.getXPosition(tracker_id);
                data.y = optitrack_.getYPosition(tracker_id);
                data.z = optitrack_.getZPosition(tracker_id);
                data.yaw_raw = optitrack_.getRawYaw(tracker_id);
                data.yaw_corrected = optitrack_.getYaw(tracker_id);
                data.commanded_yaw = 0.0; // No yaw command
                calibration_logger.logData(data);
                
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
            
            std::this_thread::sleep_for(std::chrono::seconds(1)); // Stabilize
            
            // 3. Move forward for 5 seconds
            std::cout << "Step 3: Moving forward for 5 seconds...\n";
            tello_controller_.sendCommand(drone_ip, "forward 50");
            
            // Initialize variables to store yaw values
            std::vector<double> yaw_values;
            double initial_yaw = optitrack_.getYaw(tracker_id);
            
            // Log positions during forward movement (50 samples over 5 seconds)
            for (int i = 0; i < 50; i++) {
                DataPoint data(tracker_id);
                data.x = optitrack_.getXPosition(tracker_id);
                data.y = optitrack_.getYPosition(tracker_id);
                data.z = optitrack_.getZPosition(tracker_id);
                data.yaw_raw = optitrack_.getRawYaw(tracker_id);
                data.yaw_corrected = optitrack_.getYaw(tracker_id);
                data.commanded_yaw = 0.0; // Forward with no yaw
                calibration_logger.logData(data);
                
                // Store yaw values for calibration during the middle 3 seconds
                if (i >= 10 && i < 40) {
                    yaw_values.push_back(data.yaw_corrected);
                }
                
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
            
            // 4. Land the drone
            std::cout << "Step 4: Landing...\n";
            tello_controller_.sendCommand(drone_ip, "land");
            std::this_thread::sleep_for(std::chrono::seconds(5));
            
            // 5. Calculate yaw offset
            std::cout << "Step 5: Calculating yaw offset...\n";
            
            // Calculate average yaw during forward flight
            double sum_sin = 0, sum_cos = 0;
            for (double yaw : yaw_values) {
                // Convert to radians for the calculation
                double yaw_rad = yaw * M_PI / 180.0;
                sum_sin += std::sin(yaw_rad);
                sum_cos += std::cos(yaw_rad);
            }
            
            double avg_yaw = 0.0;
            if (!yaw_values.empty()) {
                avg_yaw = std::atan2(sum_sin / yaw_values.size(), 
                                    sum_cos / yaw_values.size()) * 180.0 / M_PI;
            }
            
            // The forward direction in drone's frame is the calibration reference
            // The offset is what we need to add to OptiTrack yaw to align with drone's frame
            
            // Update the drone data in the main list (need to use mutex if we modify shared data)
            {
                std::lock_guard<std::mutex> lock(drones_mutex_);
                drones_[drone_idx].yaw_offset = avg_yaw;
            }
            
            // Make sure the logger flushes all data
            calibration_logger.flush();
            
            std::cout << "\n========================================\n";
            std::cout << "CALIBRATION COMPLETE FOR " << name << "\n";
            std::cout << "Yaw offset: " << avg_yaw << " degrees\n";
            std::cout << "Calibration data points: " << yaw_values.size() << "\n";
            std::cout << "Log file: " << log_filename << "\n";
            std::cout << "========================================\n";
            
            // Wait for user to acknowledge results
            std::cout << "Press Enter to return to main menu...";
            std::cin.get();
            
            // Return to main menu
            display();
            
        } catch (const std::exception& e) {
            std::cout << "ERROR during calibration: " << e.what() << "\n";
            
            // Try to land the drone
            tello_controller_.sendCommand(drone_ip, "land");
            
            // Flush logger
            calibration_logger.flush();
            
            // Wait for user acknowledgment 
            std::cout << "Press Enter to return to main menu...";
            std::cin.get();
            
            // Return to main menu
            display();
        }
    }).detach();
}

void Menu::handleRebootAllDrones() {
    rebootAllDrones(true, false);
    display(); // Show main menu again
}

void Menu::handleExit() {
    std::cout << "Exiting program...\n";
    
    // Perform cleanup before exit
    LOG_INFO("Performing cleanup before exit");
    rebootAllDrones(false, true);
    
    // Exit the program
    exit(0);
}