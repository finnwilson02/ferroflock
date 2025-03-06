#ifndef MENU_H
#define MENU_H

#include <string>
#include <vector>
#include <functional>
#include <iostream>
#include <map>
#include <mutex>
#include "optitrack.h"
#include "tello_controller.h"
#include "calibration.h"

// Menu option struct
struct MenuOption {
    std::string label;
    std::function<void()> action;
    std::string description;
};

class Menu {
public:
    // Constructor/Destructor
    Menu(OptiTrack& optitrack, TelloController& tello_controller);
    ~Menu();
    
    // Display the menu and get user choice
    void display();
    
    // Display the main menu
    void displayMainMenu();
    
    // Display a list of available drones
    void displayDroneList();
    
    // Add a menu option
    void addOption(const std::string& key, const std::string& label, 
                   std::function<void()> action, const std::string& description = "");
    
    // Remove a menu option
    void removeOption(const std::string& key);
    
    // Execute an action associated with a key
    bool executeOption(const std::string& key);
    
    // Set the prompt text
    void setPrompt(const std::string& prompt);
    
    // Set the header text
    void setHeader(const std::string& header);
    
    // Set the footer text
    void setFooter(const std::string& footer);
    
    // Find drone by IP address
    DroneData* findDroneByIP(const std::string& ip);
    
    // Load drones from a JSON file
    std::vector<DroneData> loadDronesFromJSON(const std::string& filename);
    
    // Initialize the menu system
    void initialize();
    
    // Define default menu actions
    void defineDefaultActions();
    
    // Function to reboot all drones
    void rebootAllDrones(bool wait_for_reboot = true, bool send_land_first = false);

private:
    // Reference to OptiTrack system
    OptiTrack& optitrack_;
    
    // Reference to TelloController
    TelloController& tello_controller_;
    
    // Reference to Calibration system
    Calibration* calibration_;
    
    // Map of option keys to MenuOption objects
    std::map<std::string, MenuOption> options_;
    
    // Header, prompt, and footer text
    std::string header_{"===== Menu =====\n"};
    std::string prompt_{"Enter choice: "};
    std::string footer_{"\n"};
    
    // Available drones
    std::vector<DroneData> drones_;
    
    // Mutex for thread safety when accessing drones_
    std::mutex drones_mutex_;
    
    // Get user input with validation
    std::string getValidInput(const std::vector<std::string>& valid_inputs);
    
    // Action handlers
    void handleListDrones();
    void handleNetworkScan();
    void handleMapOptiTrack();
    void handleCalibrateDrone();
    void handleRebootAllDrones();
    void handleExit();
};

#endif // MENU_H