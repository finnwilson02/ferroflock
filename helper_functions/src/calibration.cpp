/**
 * calibration.cpp
 * 
 * Purpose: Implementation of the Calibration class for drone calibration
 * 
 * Data Flow:
 *   Input: OptiTrack position/orientation data, drone responses
 *   Output: Calibrated parameters (e.g., yaw offset) for accurate control
 * 
 * This implementation handles the drone orientation calibration process,
 * synchronizing OptiTrack data with drone orientation to establish accurate
 * movement mapping.
 */

#include "../include/calibration.h"
#include "../include/tello_imu_handler.h"
#include <iostream>
#include <filesystem>
#include <iomanip>

// Constructor
Calibration::Calibration(OptiTrack& optitrack, TelloController& tello_controller)
    : optitrack_(optitrack), tello_controller_(tello_controller) {
    LOG_INFO("Initializing calibration system");
}

// Destructor
Calibration::~Calibration() {
    LOG_INFO("Shutting down calibration system");
    
    // Make sure to stop any running calibration thread
    if (calibration_in_progress_) {
        LOG_WARNING("Calibration still in progress during destruction, waiting for it to finish");
        
        if (calibration_thread_.joinable()) {
            calibration_thread_.join();
        }
    }
}

// Start the calibration routine for a specific drone
void Calibration::calibrateDroneOrientation(DroneData& drone, int drone_idx, std::function<void()> on_complete_callback) {
    // Check if a calibration is already in progress
    {
        std::lock_guard<std::mutex> lock(calibration_mutex_);
        if (calibration_in_progress_) {
            LOG_WARNING("Calibration already in progress, cannot start a new one");
            std::cout << "A calibration is already in progress. Please wait for it to complete.\n";
            if (on_complete_callback) {
                on_complete_callback();
            }
            return;
        }
        
        // Set the flag to indicate that a calibration is in progress
        calibration_in_progress_ = true;
    }
    
    // Check if the drone is online
    if (!tello_controller_.pingDrone(drone.ip)) {
        LOG_ERROR("Drone " + drone.name + " is not online, cannot calibrate");
        std::cout << "Error: Drone " << drone.name << " is not online. Cannot calibrate.\n";
        
        // Reset the flag
        calibration_in_progress_ = false;
        
        if (on_complete_callback) {
            on_complete_callback();
        }
        return;
    }
    
    // Initialize drone for SDK mode
    if (!tello_controller_.initialize(drone.ip, false)) {
        LOG_ERROR("Failed to initialize drone at " + drone.ip);
        std::cout << "Error: Failed to initialize drone at " << drone.ip << ".\n";
        
        // Reset the flag
        calibration_in_progress_ = false;
        
        if (on_complete_callback) {
            on_complete_callback();
        }
        return;
    }
    
    // Check if the tracker is visible in OptiTrack before proceeding
    if (!optitrack_.isTrackerActive(drone.tracker_id)) {
        std::cout << "[ERROR] Tracker " << drone.tracker_id << " not visible in OptiTrack. Calibration may fail." << std::endl;
        LOG_WARNING("Tracker " + drone.tracker_id + " not active before calibration");
    } else {
        std::cout << "[DEBUG] Tracker " << drone.tracker_id << " is visible, proceeding with calibration" << std::endl;
    }
    
    // Make sure data directory exists (relative to the executable)
    std::string data_dir = "../data";
    Logger::createDataDirectory(data_dir);
    
    // Create a logger to record calibration data
    std::string log_filename = data_dir + "/" + Logger::createUniqueFilename("application_log", ".csv");
    Logger calibration_logger(log_filename);
    
    if (!calibration_logger.isOpen()) {
        LOG_ERROR("Failed to open log file: " + log_filename);
        std::cout << "Error: Failed to open log file. Aborting calibration.\n";
        
        // Reset the flag
        calibration_in_progress_ = false;
        
        if (on_complete_callback) {
            on_complete_callback();
        }
        return;
    }
    
    LOG_INFO("Starting calibration for drone " + drone.name + " (IP: " + drone.ip + ")");
    std::cout << "\nStarting calibration for " << drone.name << " (" << drone.ip << ")\n";
    std::cout << "Starting calibration routine...\n";
    
    // Start a new thread for the calibration routine
    if (calibration_thread_.joinable()) {
        calibration_thread_.join();
    }
    
    // Create a new std::mutex that will be shared with the thread
    auto drones_mutex = std::make_shared<std::mutex>();
    
    // Initialize IMU handler
    TelloIMUHandler imu_handler(tello_controller_, drone.ip);
    if (!imu_handler.initialize()) {
        LOG_ERROR("Failed to initialize IMU handler for " + drone.ip);
        std::cout << "Error: Failed to initialize IMU handler for " << drone.ip << ".\n";
        calibration_in_progress_ = false;
        if (on_complete_callback) on_complete_callback();
        return;
    }
    
    // Create a shared_ptr to store the drone data
    auto drones_data = std::make_shared<std::vector<DroneData>>();
    drones_data->push_back(drone);
    
    calibration_thread_ = std::thread(&Calibration::runCalibrationRoutine, this,
                               drone.ip, drone.tracker_id, drone.name, drone_idx, log_filename,
                               std::ref(calibration_logger), std::ref(*drones_data), 
                               std::ref(*drones_mutex),
                               on_complete_callback);
    
    // Detach the thread so it runs independently
    calibration_thread_.detach();
}

// Check if a calibration is in progress
bool Calibration::isCalibrationInProgress() const {
    std::lock_guard<std::mutex> lock(const_cast<std::mutex&>(calibration_mutex_));
    return calibration_in_progress_;
}

// The actual calibration routine (runs in a separate thread)
void Calibration::runCalibrationRoutine(
    const std::string& drone_ip, 
    const std::string& tracker_id,
    const std::string& drone_name,
    int drone_idx, 
    const std::string& log_filename,
    Logger& calibration_logger,
    std::vector<DroneData>& drones,
    std::mutex& drones_mutex,
    std::function<void()> on_complete_callback) {
    
    // Create a local IMU handler instance for thread safety
    TelloIMUHandler imu_handler(tello_controller_, drone_ip);
    imu_handler.initialize();
    
    try {
        // Wait a moment for OptiTrack to detect the drone
        std::this_thread::sleep_for(std::chrono::seconds(2));
        
        // Check if the tracker is visible before proceeding
        if (!optitrack_.isTrackerActive(tracker_id)) {
            std::cout << "[ERROR] Tracker " << tracker_id << " not visible before takeoff. Aborting." << std::endl;
            calibration_in_progress_ = false;
            if (on_complete_callback) on_complete_callback();
            return;
        }
        
        // 1. Takeoff
        LOG_INFO("Calibration step 1: Taking off");
        std::cout << "Step 1: Taking off...\n";
        tello_controller_.sendCommand(drone_ip, "command");
        calibration_logger.logCommand("command", 1.0, std::chrono::system_clock::to_time_t(std::chrono::system_clock::now()));
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        tello_controller_.sendCommand(drone_ip, "takeoff");
        calibration_logger.logCommand("takeoff", 1.0, std::chrono::system_clock::to_time_t(std::chrono::system_clock::now()));
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        
        // Wait for takeoff to complete
        std::this_thread::sleep_for(std::chrono::seconds(5));
        
        // Log initial position
        DataPoint takeoff_data(tracker_id);
        takeoff_data.x = optitrack_.getXPosition(tracker_id);
        takeoff_data.y = optitrack_.getYPosition(tracker_id);
        takeoff_data.z = optitrack_.getZPosition(tracker_id);
        takeoff_data.yaw_raw = optitrack_.getRawYaw(tracker_id);
        takeoff_data.yaw_corrected = optitrack_.getYaw(tracker_id);
        takeoff_data.imu_yaw = imu_handler.getYaw();
        takeoff_data.imu_pitch = imu_handler.getPitch();
        takeoff_data.imu_roll = imu_handler.getRoll();
        takeoff_data.imu_agx = imu_handler.getAgx();
        takeoff_data.imu_agy = imu_handler.getAgy();
        takeoff_data.imu_agz = imu_handler.getAgz();
        calibration_logger.logData(takeoff_data);
        std::cout << "[DEBUG] Logged data for " << tracker_id << ": x=" << takeoff_data.x 
                  << ", y=" << takeoff_data.y << ", z=" << takeoff_data.z 
                  << ", yaw=" << takeoff_data.yaw_corrected << std::endl;
        
        // 2. Move upward for 3 seconds
        LOG_INFO("Calibration step 2: Moving upward for 3 seconds");
        std::cout << "Step 2: Moving upward for 3 seconds...\n";
        tello_controller_.sendCommand(drone_ip, "up 50");
        calibration_logger.logCommand("up", 50.0, std::chrono::system_clock::to_time_t(std::chrono::system_clock::now()));
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        
        // Log positions while moving upward (30 samples over 3 seconds)
        for (int i = 0; i < 30; i++) {
            DataPoint data(tracker_id);
            data.x = optitrack_.getXPosition(tracker_id);
            data.y = optitrack_.getYPosition(tracker_id);
            data.z = optitrack_.getZPosition(tracker_id);
            data.yaw_raw = optitrack_.getRawYaw(tracker_id);
            data.yaw_corrected = optitrack_.getYaw(tracker_id);
            data.imu_yaw = imu_handler.getYaw();
            data.imu_pitch = imu_handler.getPitch();
            data.imu_roll = imu_handler.getRoll();
            data.imu_agx = imu_handler.getAgx();
            data.imu_agy = imu_handler.getAgy();
            data.imu_agz = imu_handler.getAgz();
            calibration_logger.logData(data);
            
            // Only print every 5th sample to avoid too much output
            if (i % 5 == 0) {
                std::cout << "[DEBUG] Logged data for " << tracker_id << ": x=" << data.x 
                          << ", y=" << data.y << ", z=" << data.z 
                          << ", yaw=" << data.yaw_corrected << std::endl;
            }
            
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        
        std::this_thread::sleep_for(std::chrono::seconds(1)); // Stabilize
        
        // 3. Move forward for 5 seconds
        LOG_INFO("Calibration step 3: Moving forward for 5 seconds");
        std::cout << "Step 3: Moving forward for 5 seconds...\n";
        tello_controller_.sendCommand(drone_ip, "forward 50");
        calibration_logger.logCommand("forward", 50.0, std::chrono::system_clock::to_time_t(std::chrono::system_clock::now()));
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        
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
            data.imu_yaw = imu_handler.getYaw();
            data.imu_pitch = imu_handler.getPitch();
            data.imu_roll = imu_handler.getRoll();
            data.imu_agx = imu_handler.getAgx();
            data.imu_agy = imu_handler.getAgy();
            data.imu_agz = imu_handler.getAgz();
            calibration_logger.logData(data);
            
            // Store yaw values for calibration during the middle 3 seconds
            if (i >= 10 && i < 40) {
                yaw_values.push_back(data.yaw_corrected);
            }
            
            // Only print every 10th sample to avoid too much output
            if (i % 10 == 0) {
                std::cout << "[DEBUG] Logged data for " << tracker_id << ": x=" << data.x 
                          << ", y=" << data.y << ", z=" << data.z 
                          << ", yaw=" << data.yaw_corrected << std::endl;
            }
            
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        
        // 4. Land the drone
        LOG_INFO("Calibration step 4: Landing");
        std::cout << "Step 4: Landing...\n";
        tello_controller_.sendCommand(drone_ip, "land");
        calibration_logger.logCommand("land", 1.0, std::chrono::system_clock::to_time_t(std::chrono::system_clock::now()));
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        std::this_thread::sleep_for(std::chrono::seconds(5));
        
        // 5. Calculate yaw offset
        LOG_INFO("Calibration step 5: Calculating yaw offset");
        std::cout << "Step 5: Calculating yaw offset...\n";
        
        // Print debug info about collected yaw values
        std::cout << "[DEBUG] Yaw values collected: " << yaw_values.size() << std::endl;
        if (yaw_values.empty()) {
            std::cout << "[ERROR] No yaw data collected for " << tracker_id << std::endl;
        }
        
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
            std::lock_guard<std::mutex> lock(drones_mutex);
            drones[drone_idx].yaw_offset = avg_yaw;
            
            // Also save to a calibration file
            LOG_INFO("Saving calibration data to file");
        }
        
        // Make sure the logger flushes all data
        calibration_logger.flush();
        
        LOG_INFO("Calibration complete for " + drone_name + " with yaw offset " + 
                 std::to_string(avg_yaw) + " degrees");
        
        std::cout << "\n========================================\n";
        std::cout << "CALIBRATION COMPLETE FOR " << drone_name << "\n";
        std::cout << "Yaw offset: " << avg_yaw << " degrees\n";
        std::cout << "Calibration data points: " << yaw_values.size() << "\n";
        std::cout << "Log file: " << log_filename << "\n";
        std::cout << "========================================\n";
        
        // Signal that the calibration is complete
        {
            std::lock_guard<std::mutex> lock(calibration_mutex_);
            calibration_in_progress_ = false;
        }
        
        // Call the callback function once the calibration is complete
        if (on_complete_callback) {
            on_complete_callback();
        }
        
    } catch (const std::exception& e) {
        LOG_ERROR("Exception during calibration: " + std::string(e.what()));
        std::cout << "ERROR during calibration: " << e.what() << "\n";
        
        // Try to land the drone
        tello_controller_.sendCommand(drone_ip, "land");
        calibration_logger.logCommand("land", 1.0, std::chrono::system_clock::to_time_t(std::chrono::system_clock::now()));
        
        // Flush logger
        calibration_logger.flush();
        
        // Signal that the calibration is complete (with error)
        {
            std::lock_guard<std::mutex> lock(calibration_mutex_);
            calibration_in_progress_ = false;
        }
        
        // Call the callback function
        if (on_complete_callback) {
            on_complete_callback();
        }
    }
}