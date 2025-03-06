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
    std::ifstream json_file(filename);
    
    if (!json_file.is_open()) {
        LOG_ERROR("Failed to open " + filename);
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
    
    // Read the entire file into a string
    std::stringstream buffer;
    buffer << json_file.rdbuf();
    std::string content = buffer.str();
    
    // Try to parse as JSON first
    try {
        LOG_INFO("Attempting to parse " + filename + " as JSON with devices array");
        
        // Look for the "devices" array in the JSON
        std::regex devices_pattern("\"devices\"\\s*:\\s*\\[(.*?)\\]", std::regex::multiline);
        std::smatch devices_match;
        
        if (std::regex_search(content, devices_match, devices_pattern) && devices_match.size() > 1) {
            LOG_INFO("Found devices array in JSON");
            
            // Parse individual device objects
            std::string devices_content = devices_match[1].str();
            std::regex device_pattern("\\{(.*?)\\}", std::regex::multiline);
            
            auto device_begin = std::sregex_iterator(devices_content.begin(), devices_content.end(), device_pattern);
            auto device_end = std::sregex_iterator();
            
            int drone_count = 0;
            for (std::sregex_iterator i = device_begin; i != device_end; ++i) {
                std::smatch match = *i;
                if (match.size() > 1) {
                    std::string device_str = match[0].str();
                    
                    // Extract IP
                    std::regex ip_regex("\"ip\"\\s*:\\s*\"([^\"]+)\"");
                    std::smatch ip_match;
                    std::string ip;
                    
                    if (std::regex_search(device_str, ip_match, ip_regex) && ip_match.size() > 1) {
                        ip = ip_match[1].str();
                    } else {
                        LOG_WARNING("Device without IP address found, skipping");
                        continue;
                    }
                    
                    // Extract tracker_id if present
                    std::regex tracker_regex("\"tracker_id\"\\s*:\\s*\"([^\"]+)\"");
                    std::smatch tracker_match;
                    std::string tracker_id;
                    
                    if (std::regex_search(device_str, tracker_match, tracker_regex) && tracker_match.size() > 1) {
                        tracker_id = tracker_match[1].str();
                    }
                    
                    // Extract name if present
                    std::regex name_regex("\"name\"\\s*:\\s*\"([^\"]+)\"");
                    std::smatch name_match;
                    std::string name;
                    
                    if (std::regex_search(device_str, name_match, name_regex) && name_match.size() > 1) {
                        name = name_match[1].str();
                    }
                    
                    DroneData drone;
                    drone.ip = ip;
                    
                    // Set name (use extracted value or generate one)
                    if (!name.empty()) {
                        drone.name = name;
                    } else {
                        drone.name = "Drone " + std::to_string(++drone_count);
                    }
                    
                    // Set tracker ID (use extracted value, lookup by IP, or generate one)
                    if (!tracker_id.empty()) {
                        drone.tracker_id = tracker_id;
                    } else if (ipToOptitrackMap.find(ip) != ipToOptitrackMap.end()) {
                        drone.tracker_id = ipToOptitrackMap[ip];
                        // Also use the tracker name as the drone name for consistency
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
            
            // If we found devices, return them
            if (!drones.empty()) {
                return drones;
            }
        }
        
        // If we failed to parse the devices array, fall back to the simple IP-based parsing
        LOG_WARNING("Failed to parse devices array, falling back to IP extraction");
    } catch (const std::exception& e) {
        LOG_WARNING("Error parsing JSON: " + std::string(e.what()) + ". Falling back to IP extraction.");
    }
    
    // Fallback: Just parse IP addresses from the file
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
    
    // First try to load drones from drone_info directory (in parent dir, not build)
    LOG_INFO("Checking for drones in ../drone_info/dji_devices.json");
    if (std::filesystem::exists("../drone_info/dji_devices.json")) {
        drones_ = loadDronesFromJSON("../drone_info/dji_devices.json");
    }
    
    // If no drones found, try the parent directory
    if (drones_.empty()) {
        LOG_INFO("No drones found in ../drone_info, trying ../dji_devices.json");
        drones_ = loadDronesFromJSON("../dji_devices.json");
    }
    
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
    std::cout << "Running network scan for Tello drones...\n";
    
    // Create drone_info directory if it doesn't exist (in parent directory, not in build)
    std::filesystem::create_directory("../drone_info");
    
    // Run the Python script to scan for drones (use relative paths)
    std::string command = "python3 ../python/get_devices.py --output-dir ../drone_info";
    std::cout << "Executing: " << command << "\n";
    
    int result = system(command.c_str());
    
    if (result != 0) {
        std::cout << "Error running Python script. Return code: " << result << "\n";
        display();
        return;
    }
    
    std::cout << "Network scan complete.\n";
    std::cout << "Loading updated drone information...\n";
    
    // Load the updated drone information from the parent directory
    drones_ = loadDronesFromJSON("../drone_info/dji_devices.json");
    
    // Update OptiTrack drone mapping
    optitrack_.setupDroneMapping(drones_);
    
    std::cout << "Loaded " << drones_.size() << " drones from scan.\n";
    std::cout << "Press Enter to continue...";
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    std::cin.get();
    
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