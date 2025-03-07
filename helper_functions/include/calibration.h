/**
 * calibration.h
 * 
 * Purpose: Handles calibration procedures for the drone system
 * 
 * Data Flow:
 *   Input: Position and orientation data from OptiTrack, commands to drones
 *   Output: Calibrated offsets and parameters for accurate drone control
 * 
 * This module manages calibration procedures to align drone orientation with
 * OptiTrack coordinate system and determine necessary control parameters.
 */

#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <string>
#include <vector>
#include <thread>
#include <chrono>
#include <mutex>
#include <cmath>
#include <memory>
#include "optitrack.h"
#include "tello_controller.h"
#include "logger.h"

class Calibration {
public:
    // Constructor/Destructor
    Calibration(OptiTrack& optitrack, TelloController& tello_controller);
    ~Calibration();
    
    // Start the calibration routine for a specific drone
    void calibrateDroneOrientation(DroneData& drone, int drone_idx, std::function<void()> on_complete_callback);
    
    // Check if a calibration is in progress
    bool isCalibrationInProgress() const;
    
private:
    // Reference to OptiTrack system
    OptiTrack& optitrack_;
    
    // Reference to TelloController
    TelloController& tello_controller_;
    
    // Flag to indicate if a calibration is in progress
    bool calibration_in_progress_{false};
    std::mutex calibration_mutex_;
    
    // Thread for the calibration process
    std::thread calibration_thread_;
    
    // Shared resources for the calibration thread
    std::shared_ptr<Logger> calibration_logger_;
    std::shared_ptr<std::vector<DroneData>> drones_data_;
    std::shared_ptr<std::mutex> drones_mutex_;
    
    // The actual calibration routine (runs in a separate thread)
    void runCalibrationRoutine(
        const std::string& drone_ip, 
        const std::string& tracker_id,
        const std::string& drone_name,
        int drone_idx, 
        const std::string& log_filename,
        std::shared_ptr<Logger> calibration_logger,
        std::shared_ptr<std::vector<DroneData>> drones_data,
        std::shared_ptr<std::mutex> drones_mutex,
        std::function<void()> on_complete_callback
    );
};

#endif // CALIBRATION_H