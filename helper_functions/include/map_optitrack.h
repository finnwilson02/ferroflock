/**
 * map_optitrack.h
 * 
 * Purpose: Maps OptiTrack trackers to Tello drones based on movement detection
 * 
 * Data Flow:
 *   Input: OptiTrack tracker data, Tello drone connectivity
 *   Output: Tracker-to-drone mappings saved to JSON configuration file
 * 
 * This module handles automatic detection and mapping of OptiTrack trackers
 * to physical Tello drones by flying a selected drone and detecting which
 * tracker shows the most movement.
 */

#ifndef MAP_OPTITRACK_H
#define MAP_OPTITRACK_H

#include "optitrack.h"
#include "tello_controller.h"
#include "keyboard_control.h"
#include <string>
#include <vector>
#include <map>
#include <chrono>
#include <optional>

// Structure to hold position data with timestamps
struct Position {
    double x, y, z;
    std::chrono::system_clock::time_point timestamp;
};

class OptiTrackMapper {
public:
    // Constructor takes references to OptiTrack, TelloController, and drones vector
    OptiTrackMapper(OptiTrack& optitrack, TelloController& controller, 
                   std::vector<DroneData>& drones);
    
    // Main mapping function - returns the IP and tracker ID if mapping is successful
    std::optional<std::pair<std::string, std::string>> startMapping();
    
    // Save updated drone mappings to JSON file
    bool saveDronesToJSON(const std::string& filename);

private:
    // References to system components
    OptiTrack& optitrack_;
    TelloController& controller_;
    std::vector<DroneData>& drones_;
    
    // Helper functions
    std::string getValidInput(const std::vector<std::string>& valid_inputs);
    void displayTrackerStatus();
    void displayDroneStatus();
};

#endif // MAP_OPTITRACK_H