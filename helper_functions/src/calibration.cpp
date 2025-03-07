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
    : optitrack_(optitrack), tello_controller_(tello_controller),
      calibration_logger_(nullptr), drones_data_(std::make_shared<std::vector<DroneData>>()),
      drones_mutex_(std::make_shared<std::mutex>()) {
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
    // shared_ptr members will automatically clean up when this object is destroyed
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
    
    // Create and assign the logger as a shared_ptr
    calibration_logger_ = std::make_shared<Logger>(log_filename);
    
    if (!calibration_logger_->isOpen()) {
        LOG_ERROR("Failed to open log file: " + log_filename);
        std::cout << "Error: Failed to open log file. Aborting calibration.\n";
        
        // Reset the flag
        calibration_in_progress_ = false;
        
        if (on_complete_callback) {
            on_complete_callback();
        }
        return;
    }
    
    // Add drone to drones_data_ if not already present
    {
        std::lock_guard<std::mutex> lock(*drones_mutex_);
        auto it = std::find_if(drones_data_->begin(), drones_data_->end(),
            [&drone](const DroneData& d) { return d.ip == drone.ip; });
        if (it == drones_data_->end()) {
            drones_data_->push_back(drone);
        } else {
            *it = drone; // Update existing entry
        }
    }
    
    LOG_INFO("Starting calibration for drone " + drone.name + " (IP: " + drone.ip + ")");
    std::cout << "\nStarting calibration for " << drone.name << " (" << drone.ip << ")\n";
    std::cout << "Starting calibration routine...\n";
    
    // Start a new thread for the calibration routine
    if (calibration_thread_.joinable()) {
        calibration_thread_.join();
    }
    
    // Set thread attributes to avoid pthread priority issues
    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED); // Use explicit scheduling
    pthread_attr_setschedpolicy(&attr, SCHED_OTHER); // Default scheduling policy
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED); // Ensure detached state
    
    calibration_thread_ = std::thread(&Calibration::runCalibrationRoutine, this,
        drone.ip, drone.tracker_id, drone.name, drone_idx, log_filename,
        calibration_logger_, drones_data_, drones_mutex_, on_complete_callback);
    
    // Detach the thread so it runs independently
    calibration_thread_.detach();
    
    pthread_attr_destroy(&attr);
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
    std::shared_ptr<Logger> calibration_logger,
    std::shared_ptr<std::vector<DroneData>> drones_data,
    std::shared_ptr<std::mutex> drones_mutex,
    std::function<void()> on_complete_callback) {
    
    std::unique_ptr<TelloIMUHandler> imu_handler;
    
    try {
        // Create a local IMU handler instance for thread safety
        LOG_INFO("Creating IMU handler for drone " + drone_ip);
        imu_handler = std::make_unique<TelloIMUHandler>(tello_controller_, drone_ip);
        
        if (!imu_handler->initialize()) {
            LOG_ERROR("Failed to initialize IMU handler for " + drone_ip);
            std::cout << "[ERROR] Failed to initialize IMU handler. Aborting calibration." << std::endl;
            calibration_in_progress_ = false;
            if (on_complete_callback) on_complete_callback();
            return;
        }
        
        LOG_INFO("Successfully initialized IMU handler for " + drone_ip);
        
        // Wait a moment for OptiTrack to detect the drone
        std::this_thread::sleep_for(std::chrono::seconds(2));
        
        // Check if the tracker is visible before proceeding
        if (!optitrack_.isTrackerActive(tracker_id)) {
            std::cout << "[WARNING] Tracker " << tracker_id << " not visible before takeoff. Proceeding with calibration." << std::endl;
            LOG_WARNING("Tracker " + tracker_id + " not active before takeoff");
        }
        
        // Log initial state before takeoff
        LOG_INFO("Logging initial state before takeoff");
        DataPoint initial_data(tracker_id);
        bool tracker_visible = optitrack_.isTrackerActive(tracker_id);
        
        // Handle tracker data - set zeros if not visible
        if (tracker_visible) {
            initial_data.x = optitrack_.getXPosition(tracker_id);
            initial_data.y = optitrack_.getYPosition(tracker_id);
            initial_data.z = optitrack_.getZPosition(tracker_id);
            initial_data.yaw_raw = optitrack_.getRawYaw(tracker_id);
            initial_data.yaw_corrected = optitrack_.getYaw(tracker_id);
            LOG_INFO("Initial tracker position: x=" + std::to_string(initial_data.x) + 
                    ", y=" + std::to_string(initial_data.y) + 
                    ", z=" + std::to_string(initial_data.z));
        } else {
            // Explicitly set position and orientation to zero when tracker isn't visible
            initial_data.x = 0.0;
            initial_data.y = 0.0;
            initial_data.z = 0.0;
            initial_data.yaw_raw = 0.0;
            initial_data.yaw_corrected = 0.0;
            LOG_WARNING("Tracker " + tracker_id + " not visible before takeoff, logging initial position as (0, 0, 0) and yaw as 0");
            std::cout << "[WARNING] Tracker " << tracker_id << " not visible before takeoff, logging initial position as (0, 0, 0) and yaw as 0" << std::endl;
        }
        
        // IMU data is independent of tracker visibility
        initial_data.imu_yaw = imu_handler->getYaw();
        initial_data.imu_pitch = imu_handler->getPitch();
        initial_data.imu_roll = imu_handler->getRoll();
        initial_data.imu_agx = imu_handler->getAgx();
        initial_data.imu_agy = imu_handler->getAgy();
        initial_data.imu_agz = imu_handler->getAgz();
        
        {
            static std::mutex logger_mutex;
            std::lock_guard<std::mutex> lock(logger_mutex);
            calibration_logger->logData(initial_data);
        }
        
        // 1. Takeoff
        LOG_INFO("Calibration step 1: Taking off");
        std::cout << "Step 1: Taking off...\n";
        
        // Use a mutex to protect the access to global logger during command execution
        {
            static std::mutex logger_mutex;
            std::lock_guard<std::mutex> lock(logger_mutex);
            tello_controller_.sendCommand(drone_ip, "command");
            calibration_logger->logCommand("command", 1.0, std::chrono::system_clock::to_time_t(std::chrono::system_clock::now()));
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        
        {
            static std::mutex logger_mutex;
            std::lock_guard<std::mutex> lock(logger_mutex);
            tello_controller_.sendCommand(drone_ip, "takeoff");
            calibration_logger->logCommand("takeoff", 1.0, std::chrono::system_clock::to_time_t(std::chrono::system_clock::now()));
        }
        
        // Wait for takeoff to complete while logging data
        LOG_INFO("Waiting for takeoff to complete while logging");
        auto start_time = std::chrono::steady_clock::now();
        while (std::chrono::steady_clock::now() - start_time < std::chrono::seconds(5)) {
            // Check and log tracker visibility
            bool tracker_visible = optitrack_.isTrackerActive(tracker_id);
            if (tracker_visible) {
                LOG_INFO("Tracker " + tracker_id + " is visible during takeoff");
            } else {
                LOG_WARNING("Tracker " + tracker_id + " not visible during takeoff");
            }
            
            DataPoint takeoff_data(tracker_id);
            
            // Handle tracker data - set zeros if not visible
            if (tracker_visible) {
                takeoff_data.x = optitrack_.getXPosition(tracker_id);
                takeoff_data.y = optitrack_.getYPosition(tracker_id);
                takeoff_data.z = optitrack_.getZPosition(tracker_id);
                takeoff_data.yaw_raw = optitrack_.getRawYaw(tracker_id);
                takeoff_data.yaw_corrected = optitrack_.getYaw(tracker_id);
            } else {
                // Explicitly set position and orientation to zero when tracker isn't visible
                takeoff_data.x = 0.0;
                takeoff_data.y = 0.0;
                takeoff_data.z = 0.0;
                takeoff_data.yaw_raw = 0.0;
                takeoff_data.yaw_corrected = 0.0;
                std::cout << "[WARNING] Tracker " << tracker_id << " not visible, logging position as (0, 0, 0) and yaw as 0" << std::endl;
            }
            
            // IMU data is independent of tracker visibility
            takeoff_data.imu_yaw = imu_handler->getYaw();
            takeoff_data.imu_pitch = imu_handler->getPitch();
            takeoff_data.imu_roll = imu_handler->getRoll();
            takeoff_data.imu_agx = imu_handler->getAgx();
            takeoff_data.imu_agy = imu_handler->getAgy();
            takeoff_data.imu_agz = imu_handler->getAgz();
            
            {
                static std::mutex logger_mutex;
                std::lock_guard<std::mutex> lock(logger_mutex);
                calibration_logger->logData(takeoff_data);
            }
            
            // Log every 100ms (10Hz) during takeoff
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        
        // Wait for tracker to become visible after takeoff (up to 10 seconds)
        LOG_INFO("Waiting for tracker " + tracker_id + " to become visible after takeoff...");
        std::cout << "[INFO] Waiting for tracker " << tracker_id << " to become visible after takeoff..." << std::endl;
        
        bool tracker_became_visible = false;
        start_time = std::chrono::steady_clock::now();
        
        while (std::chrono::steady_clock::now() - start_time < std::chrono::seconds(10)) {
            if (optitrack_.isTrackerActive(tracker_id)) {
                tracker_became_visible = true;
                LOG_INFO("Tracker " + tracker_id + " is now visible. Proceeding with calibration.");
                std::cout << "[INFO] Tracker " << tracker_id << " is now visible. Proceeding with calibration." << std::endl;
                break;
            }
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        
        if (!tracker_became_visible) {
            LOG_ERROR("Tracker " + tracker_id + " not visible after takeoff. Aborting calibration.");
            std::cout << "[ERROR] Tracker " << tracker_id << " not visible after takeoff. Aborting calibration." << std::endl;
            
            // Try to land the drone
            tello_controller_.sendCommand(drone_ip, "land");
            calibration_logger->logCommand("land", 1.0, std::chrono::system_clock::to_time_t(std::chrono::system_clock::now()));
            
            // Signal that the calibration is complete (with error)
            calibration_in_progress_ = false;
            if (on_complete_callback) on_complete_callback();
            return;
        }
        
        // Log post-takeoff position
        DataPoint post_takeoff_data(tracker_id);
        post_takeoff_data.x = optitrack_.getXPosition(tracker_id);
        post_takeoff_data.y = optitrack_.getYPosition(tracker_id);
        post_takeoff_data.z = optitrack_.getZPosition(tracker_id);
        post_takeoff_data.yaw_raw = optitrack_.getRawYaw(tracker_id);
        post_takeoff_data.yaw_corrected = optitrack_.getYaw(tracker_id);
        post_takeoff_data.imu_yaw = imu_handler->getYaw();
        post_takeoff_data.imu_pitch = imu_handler->getPitch();
        post_takeoff_data.imu_roll = imu_handler->getRoll();
        post_takeoff_data.imu_agx = imu_handler->getAgx();
        post_takeoff_data.imu_agy = imu_handler->getAgy();
        post_takeoff_data.imu_agz = imu_handler->getAgz();
        
        {
            static std::mutex logger_mutex;
            std::lock_guard<std::mutex> lock(logger_mutex);
            calibration_logger->logData(post_takeoff_data);
        }
        
        std::cout << "[DEBUG] Logged data for " << tracker_id << ": x=" << post_takeoff_data.x 
                  << ", y=" << post_takeoff_data.y << ", z=" << post_takeoff_data.z 
                  << ", yaw=" << post_takeoff_data.yaw_corrected << std::endl;
        
        // 2. Move upward for 3 seconds using continuous RC commands
        LOG_INFO("Calibration step 2: Moving upward for 3 seconds");
        std::cout << "Step 2: Moving upward for 3 seconds...\n";
        
        // Log command start
        {
            static std::mutex logger_mutex;
            std::lock_guard<std::mutex> lock(logger_mutex);
            calibration_logger->logCommand("rc_upward_start", 50.0, std::chrono::system_clock::to_time_t(std::chrono::system_clock::now()));
        }
        
        // Send continuous upward commands while logging data
        start_time = std::chrono::steady_clock::now();
        int log_counter = 0;
        while (std::chrono::steady_clock::now() - start_time < std::chrono::seconds(3)) {
            // Send RC command for upward movement (left-right, forward-back, up-down, yaw)
            tello_controller_.sendCommand(drone_ip, "rc 0 0 50 0");
            
            // Check and log tracker visibility
            bool tracker_visible = optitrack_.isTrackerActive(tracker_id);
            
            // Log position data
            DataPoint data(tracker_id);
            
            // Handle tracker data - set zeros if not visible
            if (tracker_visible) {
                data.x = optitrack_.getXPosition(tracker_id);
                data.y = optitrack_.getYPosition(tracker_id);
                data.z = optitrack_.getZPosition(tracker_id);
                data.yaw_raw = optitrack_.getRawYaw(tracker_id);
                data.yaw_corrected = optitrack_.getYaw(tracker_id);
            } else {
                // Explicitly set position and orientation to zero when tracker isn't visible
                data.x = 0.0;
                data.y = 0.0;
                data.z = 0.0;
                data.yaw_raw = 0.0;
                data.yaw_corrected = 0.0;
                
                // Only log visibility warning every 10th sample to avoid too much output
                if (log_counter % 10 == 0) {
                    LOG_WARNING("Tracker " + tracker_id + " not visible during upward movement");
                    std::cout << "[WARNING] Tracker " << tracker_id << " not visible, logging position as (0, 0, 0) and yaw as 0" << std::endl;
                }
            }
            
            // IMU data is independent of tracker visibility
            data.imu_yaw = imu_handler->getYaw();
            data.imu_pitch = imu_handler->getPitch();
            data.imu_roll = imu_handler->getRoll();
            data.imu_agx = imu_handler->getAgx();
            data.imu_agy = imu_handler->getAgy();
            data.imu_agz = imu_handler->getAgz();
            
            {
                static std::mutex logger_mutex;
                std::lock_guard<std::mutex> lock(logger_mutex);
                calibration_logger->logData(data);
            }
            
            // Only print every 10th sample to avoid too much output
            if (log_counter % 10 == 0) {
                std::cout << "[DEBUG] Upward movement - logged data: x=" << data.x 
                          << ", y=" << data.y << ", z=" << data.z 
                          << ", yaw=" << data.yaw_corrected 
                          << (tracker_visible ? " (tracker visible)" : " (tracker not visible)") << std::endl;
            }
            log_counter++;
            
            // Send commands at 20Hz (50ms interval)
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        
        // Stop movement
        LOG_INFO("Stopping upward movement");
        for (int i = 0; i < 3; i++) {  // Send stop command multiple times for reliability
            tello_controller_.sendCommand(drone_ip, "rc 0 0 0 0");
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        
        // Log command end
        {
            static std::mutex logger_mutex;
            std::lock_guard<std::mutex> lock(logger_mutex);
            calibration_logger->logCommand("rc_upward_stop", 0.0, std::chrono::system_clock::to_time_t(std::chrono::system_clock::now()));
        }
        
        // Log while stabilizing for 1 second
        LOG_INFO("Stabilizing for 1 second");
        start_time = std::chrono::steady_clock::now();
        while (std::chrono::steady_clock::now() - start_time < std::chrono::seconds(1)) {
            DataPoint data(tracker_id);
            data.x = optitrack_.getXPosition(tracker_id);
            data.y = optitrack_.getYPosition(tracker_id);
            data.z = optitrack_.getZPosition(tracker_id);
            data.yaw_raw = optitrack_.getRawYaw(tracker_id);
            data.yaw_corrected = optitrack_.getYaw(tracker_id);
            data.imu_yaw = imu_handler->getYaw();
            data.imu_pitch = imu_handler->getPitch();
            data.imu_roll = imu_handler->getRoll();
            data.imu_agx = imu_handler->getAgx();
            data.imu_agy = imu_handler->getAgy();
            data.imu_agz = imu_handler->getAgz();
            
            {
                static std::mutex logger_mutex;
                std::lock_guard<std::mutex> lock(logger_mutex);
                calibration_logger->logData(data);
            }
            
            // Log at 10Hz during stabilization
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        
        // 3. Move forward for 5 seconds using continuous RC commands
        LOG_INFO("Calibration step 3: Moving forward for 5 seconds");
        std::cout << "Step 3: Moving forward for 5 seconds...\n";
        
        // Log command start
        {
            static std::mutex logger_mutex;
            std::lock_guard<std::mutex> lock(logger_mutex);
            calibration_logger->logCommand("rc_forward_start", 50.0, std::chrono::system_clock::to_time_t(std::chrono::system_clock::now()));
        }
        
        // Initialize variables to store yaw values
        std::vector<double> yaw_values;
        double initial_yaw = optitrack_.getYaw(tracker_id);
        
        // Send continuous forward commands while logging data
        start_time = std::chrono::steady_clock::now();
        log_counter = 0;
        
        // Calculate time ranges for middle 3 seconds (from 1s to 4s mark)
        auto middle_start = start_time + std::chrono::seconds(1);
        auto middle_end = start_time + std::chrono::seconds(4);
        
        while (std::chrono::steady_clock::now() - start_time < std::chrono::seconds(5)) {
            auto current_time = std::chrono::steady_clock::now();
            
            // Send RC command for forward movement (left-right, forward-back, up-down, yaw)
            tello_controller_.sendCommand(drone_ip, "rc 0 50 0 0");
            
            // Check tracker visibility
            bool tracker_visible = optitrack_.isTrackerActive(tracker_id);
            
            // Log position data
            DataPoint data(tracker_id);
            
            // Handle tracker data - set zeros if not visible
            if (tracker_visible) {
                data.x = optitrack_.getXPosition(tracker_id);
                data.y = optitrack_.getYPosition(tracker_id);
                data.z = optitrack_.getZPosition(tracker_id);
                data.yaw_raw = optitrack_.getRawYaw(tracker_id);
                data.yaw_corrected = optitrack_.getYaw(tracker_id);
            } else {
                // Explicitly set position and orientation to zero when tracker isn't visible
                data.x = 0.0;
                data.y = 0.0;
                data.z = 0.0;
                data.yaw_raw = 0.0;
                data.yaw_corrected = 0.0;
                
                // Only log visibility warning every 10th sample to avoid too much output
                if (log_counter % 10 == 0) {
                    LOG_WARNING("Tracker " + tracker_id + " not visible during forward movement");
                    std::cout << "[WARNING] Tracker " << tracker_id << " not visible, logging position as (0, 0, 0) and yaw as 0" << std::endl;
                }
            }
            
            // IMU data is independent of tracker visibility
            data.imu_yaw = imu_handler->getYaw();
            data.imu_pitch = imu_handler->getPitch();
            data.imu_roll = imu_handler->getRoll();
            data.imu_agx = imu_handler->getAgx();
            data.imu_agy = imu_handler->getAgy();
            data.imu_agz = imu_handler->getAgz();
            
            {
                static std::mutex logger_mutex;
                std::lock_guard<std::mutex> lock(logger_mutex);
                calibration_logger->logData(data);
            }
            
            // Store yaw values for calibration during the middle 3 seconds
            // Only store values when tracker is visible
            if (current_time >= middle_start && current_time <= middle_end && tracker_visible) {
                yaw_values.push_back(data.yaw_corrected);
            }
            
            // Only print every 10th sample to avoid too much output
            if (log_counter % 10 == 0) {
                std::cout << "[DEBUG] Forward movement - logged data: x=" << data.x 
                          << ", y=" << data.y << ", z=" << data.z 
                          << ", yaw=" << data.yaw_corrected 
                          << (tracker_visible ? " (tracker visible)" : " (tracker not visible)") << std::endl;
            }
            log_counter++;
            
            // Send commands at 20Hz (50ms interval)
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        
        // Stop movement
        LOG_INFO("Stopping forward movement");
        for (int i = 0; i < 3; i++) {  // Send stop command multiple times for reliability
            tello_controller_.sendCommand(drone_ip, "rc 0 0 0 0");
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        
        // Log command end
        {
            static std::mutex logger_mutex;
            std::lock_guard<std::mutex> lock(logger_mutex);
            calibration_logger->logCommand("rc_forward_stop", 0.0, std::chrono::system_clock::to_time_t(std::chrono::system_clock::now()));
        }
        
        // 4. Land the drone
        LOG_INFO("Calibration step 4: Landing");
        std::cout << "Step 4: Landing...\n";
        
        // Send land command
        {
            static std::mutex logger_mutex;
            std::lock_guard<std::mutex> lock(logger_mutex);
            tello_controller_.sendCommand(drone_ip, "land");
            calibration_logger->logCommand("land", 1.0, std::chrono::system_clock::to_time_t(std::chrono::system_clock::now()));
        }
        
        // Log data during landing process
        LOG_INFO("Logging data during landing process");
        start_time = std::chrono::steady_clock::now();
        log_counter = 0;
        while (std::chrono::steady_clock::now() - start_time < std::chrono::seconds(5)) {
            // Check tracker visibility
            bool tracker_visible = optitrack_.isTrackerActive(tracker_id);
            
            DataPoint landing_data(tracker_id);
            
            // Handle tracker data - set zeros if not visible
            if (tracker_visible) {
                landing_data.x = optitrack_.getXPosition(tracker_id);
                landing_data.y = optitrack_.getYPosition(tracker_id);
                landing_data.z = optitrack_.getZPosition(tracker_id);
                landing_data.yaw_raw = optitrack_.getRawYaw(tracker_id);
                landing_data.yaw_corrected = optitrack_.getYaw(tracker_id);
            } else {
                // Explicitly set position and orientation to zero when tracker isn't visible
                landing_data.x = 0.0;
                landing_data.y = 0.0;
                landing_data.z = 0.0;
                landing_data.yaw_raw = 0.0;
                landing_data.yaw_corrected = 0.0;
                
                // Only log visibility warning every 10th sample to avoid too much output
                if (log_counter % 10 == 0) {
                    LOG_WARNING("Tracker " + tracker_id + " not visible during landing process");
                    std::cout << "[WARNING] Tracker " << tracker_id << " not visible during landing, logging position as (0, 0, 0) and yaw as 0" << std::endl;
                }
            }
            
            // IMU data is independent of tracker visibility
            landing_data.imu_yaw = imu_handler->getYaw();
            landing_data.imu_pitch = imu_handler->getPitch();
            landing_data.imu_roll = imu_handler->getRoll();
            landing_data.imu_agx = imu_handler->getAgx();
            landing_data.imu_agy = imu_handler->getAgy();
            landing_data.imu_agz = imu_handler->getAgz();
            
            {
                static std::mutex logger_mutex;
                std::lock_guard<std::mutex> lock(logger_mutex);
                calibration_logger->logData(landing_data);
            }
            
            // Only print every 10th sample to avoid too much output
            if (log_counter % 10 == 0) {
                std::cout << "[DEBUG] Landing process - logged data: x=" << landing_data.x 
                          << ", y=" << landing_data.y << ", z=" << landing_data.z 
                          << ", yaw=" << landing_data.yaw_corrected 
                          << (tracker_visible ? " (tracker visible)" : " (tracker not visible)") << std::endl;
            }
            log_counter++;
            
            // Log at 10Hz during landing
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        
        // Log final position after landing
        DataPoint final_data(tracker_id);
        tracker_visible = optitrack_.isTrackerActive(tracker_id);
        
        // Handle tracker data - set zeros if not visible
        if (tracker_visible) {
            final_data.x = optitrack_.getXPosition(tracker_id);
            final_data.y = optitrack_.getYPosition(tracker_id);
            final_data.z = optitrack_.getZPosition(tracker_id);
            final_data.yaw_raw = optitrack_.getRawYaw(tracker_id);
            final_data.yaw_corrected = optitrack_.getYaw(tracker_id);
            LOG_INFO("Final tracker position: x=" + std::to_string(final_data.x) + 
                    ", y=" + std::to_string(final_data.y) + 
                    ", z=" + std::to_string(final_data.z));
        } else {
            // Explicitly set position and orientation to zero when tracker isn't visible
            final_data.x = 0.0;
            final_data.y = 0.0;
            final_data.z = 0.0;
            final_data.yaw_raw = 0.0;
            final_data.yaw_corrected = 0.0;
            LOG_WARNING("Tracker " + tracker_id + " not visible after landing, logging position as (0, 0, 0) and yaw as 0");
            std::cout << "[WARNING] Tracker " << tracker_id << " not visible after landing, logging position as (0, 0, 0) and yaw as 0" << std::endl;
        }
        
        // IMU data is independent of tracker visibility
        final_data.imu_yaw = imu_handler->getYaw();
        final_data.imu_pitch = imu_handler->getPitch();
        final_data.imu_roll = imu_handler->getRoll();
        final_data.imu_agx = imu_handler->getAgx();
        final_data.imu_agy = imu_handler->getAgy();
        final_data.imu_agz = imu_handler->getAgz();
        
        {
            static std::mutex logger_mutex;
            std::lock_guard<std::mutex> lock(logger_mutex);
            calibration_logger->logData(final_data);
        }
        
        LOG_INFO("Landing complete");
        std::cout << "[INFO] Landing complete" 
                  << (tracker_visible ? " with final tracker data" : " (tracker not visible at final position)") << std::endl;
        
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
            std::lock_guard<std::mutex> lock(*drones_mutex);
            if (drone_idx < drones_data->size()) {
                (*drones_data)[drone_idx].yaw_offset = avg_yaw;
            }
            
            // Also save to a calibration file
            LOG_INFO("Saving calibration data to file");
        }
        
        // Make sure the logger flushes all data
        calibration_logger->flush();
        
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
        {
            static std::mutex logger_mutex;
            std::lock_guard<std::mutex> lock(logger_mutex);
            tello_controller_.sendCommand(drone_ip, "land");
            calibration_logger->logCommand("land", 1.0, std::chrono::system_clock::to_time_t(std::chrono::system_clock::now()));
        }
        
        // Flush logger
        calibration_logger->flush();
        
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