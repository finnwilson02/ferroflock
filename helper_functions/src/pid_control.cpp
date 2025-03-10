/**
 * pid_control.cpp
 * 
 * Purpose: Implementation of PID (Proportional-Integral-Derivative) controller for drone flight
 * 
 * Data Flow:
 *   Input: Desired rates and estimated rates from UKF state vector
 *   Output: Control commands to stabilize the drone
 * 
 * This implementation handles the loading of PID gains from JSON, initializing
 * controllers for specific drones, and computing control outputs based on the
 * difference between desired and estimated rates.
 */

#include "../include/pid_control.h"
#include <fstream>
#include <iostream>
#include <cmath>
#include <filesystem>
#include <cstdlib> // For getenv

using json = nlohmann::json;

// Define paths to JSON files
static const std::string HOME_JSON_PATH = std::string(getenv("HOME")) + "/ferroflock/dji_devices.json";
static const std::string RELATIVE_JSON_PATH = "../../dji_devices.json";

/**
 * Constructor
 */
PIDController::PIDController() {
    LOG_INFO("Initializing PID controller");
    
    // Default active MAC to empty
    active_mac_ = "";
    
    // Load gains from JSON file
    loadGainsFromJSON();
}

/**
 * Initialize the PID controller for a specific drone
 * @param mac_address MAC address of the drone
 * @return True if initialization succeeded, false otherwise
 */
bool PIDController::initialize(const std::string& mac_address) {
    LOG_INFO("Initializing PID controller for drone: " + mac_address);
    
    // Check if the MAC address is empty
    if (mac_address.empty()) {
        LOG_ERROR("Cannot initialize PID controller with empty MAC address");
        return false;
    }
    
    // Make sure gains are loaded
    if (gains_.empty()) {
        LOG_INFO("No gains loaded yet, attempting to load from JSON");
        if (!loadGainsFromJSON()) {
            LOG_WARNING("Failed to load gains, using default values");
            // This is not a fatal error, we'll use default gains
        }
    }
    
    // Lock for thread safety
    std::lock_guard<std::mutex> lock(mutex_);
    
    // Set as active MAC address
    active_mac_ = mac_address;
    
    // Create default gains if they don't exist for this drone
    if (gains_.find(mac_address) == gains_.end()) {
        LOG_INFO("No specific gains found for " + mac_address + ", using defaults");
        gains_[mac_address] = Gains(); // Use default values
    }
    
    // Initialize or reset the PID state
    PIDState new_state;
    new_state.integral = Rates(0.0, 0.0, 0.0);
    new_state.prev_error = Rates(0.0, 0.0, 0.0);
    new_state.prev_time = 0.0;
    
    states_[mac_address] = new_state;
    
    LOG_INFO("PID controller initialized for drone " + mac_address + 
             " with gains: Kp=" + std::to_string(gains_[mac_address].Kp) +
             ", Ki=" + std::to_string(gains_[mac_address].Ki) +
             ", Kd=" + std::to_string(gains_[mac_address].Kd));
    
    return true;
}

/**
 * Load PID gains from JSON file
 * @param filename Path to JSON file, defaults to standard locations
 * @return True if loading succeeded, false otherwise
 */
bool PIDController::loadGainsFromJSON(const std::string& filename) {
    LOG_INFO("Loading PID gains from JSON");
    
    // Determine which file to use
    std::string actual_filename;
    
    if (!filename.empty()) {
        // Use specified file
        actual_filename = filename;
    } else if (std::filesystem::exists(HOME_JSON_PATH)) {
        // Use home directory path
        actual_filename = HOME_JSON_PATH;
        LOG_INFO("Using JSON file: " + HOME_JSON_PATH);
    } else if (std::filesystem::exists(RELATIVE_JSON_PATH)) {
        // Use relative path
        actual_filename = RELATIVE_JSON_PATH;
        LOG_INFO("Using JSON file: " + RELATIVE_JSON_PATH);
    } else {
        LOG_ERROR("No valid JSON file found. Tried: " + HOME_JSON_PATH + " and " + RELATIVE_JSON_PATH);
        return false;
    }
    
    // Open the JSON file
    std::ifstream json_file(actual_filename);
    if (!json_file.is_open()) {
        LOG_ERROR("Failed to open JSON file: " + actual_filename);
        return false;
    }
    
    // Lock for thread safety
    std::lock_guard<std::mutex> lock(mutex_);
    
    // Parse JSON
    try {
        json j;
        json_file >> j;
        
        // Check if the JSON has a "devices" array
        if (!j.contains("devices") || !j["devices"].is_array()) {
            LOG_ERROR("JSON file does not contain a 'devices' array: " + actual_filename);
            return false;
        }
        
        // Clear existing gains
        gains_.clear();
        
        // Flag to track if any gains were loaded
        bool any_gains_loaded = false;
        
        // Process each device in the array
        for (const auto& device : j["devices"]) {
            // Extract the MAC address (required)
            if (!device.contains("mac") || !device["mac"].is_string()) {
                LOG_WARNING("Device missing MAC address, skipping");
                continue;
            }
            
            std::string mac = device["mac"].get<std::string>();
            
            // Initialize default gains
            Gains gain_values;
            
            // Extract PID gains if available
            if (device.contains("Kp") && device["Kp"].is_number()) {
                gain_values.Kp = device["Kp"].get<double>();
            }
            
            if (device.contains("Ki") && device["Ki"].is_number()) {
                gain_values.Ki = device["Ki"].get<double>();
            }
            
            if (device.contains("Kd") && device["Kd"].is_number()) {
                gain_values.Kd = device["Kd"].get<double>();
            }
            
            // Store gains in the map
            gains_[mac] = gain_values;
            
            LOG_INFO("Loaded PID gains for device " + mac + 
                    ": Kp=" + std::to_string(gain_values.Kp) + 
                    ", Ki=" + std::to_string(gain_values.Ki) + 
                    ", Kd=" + std::to_string(gain_values.Kd));
            
            any_gains_loaded = true;
        }
        
        if (!any_gains_loaded) {
            LOG_WARNING("No PID gains loaded from JSON file: " + actual_filename);
        } else {
            LOG_INFO("Successfully loaded PID gains for " + std::to_string(gains_.size()) + 
                    " devices from " + actual_filename);
        }
        
        return any_gains_loaded;
        
    } catch (const json::exception& e) {
        LOG_ERROR("JSON parsing error in " + actual_filename + ": " + e.what());
        return false;
    }
}

/**
 * Compute control output based on desired rates and current state
 * @param desired_rates Desired roll, pitch, and yaw rates
 * @param state Current state vector from UKF
 * @param current_time Current time in seconds
 * @return Control commands as roll, pitch, and yaw rates
 */
PIDController::Rates PIDController::computeControl(
    const Rates& desired_rates, 
    const StateVector& state,
    double current_time) {
    
    LOG_DEBUG("Computing PID control output");
    
    // If no active MAC address is set, return zero control
    if (active_mac_.empty()) {
        LOG_WARNING("No active drone MAC address set for PID control");
        return Rates(0.0, 0.0, 0.0);
    }
    
    // Lock for thread safety
    std::lock_guard<std::mutex> lock(mutex_);
    
    // Ensure this drone exists in our state maps
    if (gains_.find(active_mac_) == gains_.end()) {
        LOG_WARNING("No gains found for active drone: " + active_mac_ + ", using defaults");
        gains_[active_mac_] = Gains(); // Use default values
    }
    
    if (states_.find(active_mac_) == states_.end()) {
        LOG_WARNING("No state found for active drone: " + active_mac_ + ", initializing");
        states_[active_mac_] = PIDState(); // Initialize state
    }
    
    // Get gains for the active drone
    const Gains& gains = gains_[active_mac_];
    PIDState& pid_state = states_[active_mac_];
    
    // Calculate time step (dt)
    double dt;
    if (pid_state.prev_time <= 0.0) {
        // First update, use default dt
        dt = DEFAULT_DT;
    } else {
        dt = current_time - pid_state.prev_time;
        
        // Sanity check on dt
        if (dt <= 0.0) {
            LOG_WARNING("Invalid time step: " + std::to_string(dt) + ", using default");
            dt = DEFAULT_DT;
        } else if (dt > 1.0) {
            // If dt is too large (more than 1 second), cap it to avoid large integral jumps
            LOG_WARNING("Large time step: " + std::to_string(dt) + ", capping to 1.0");
            dt = 1.0;
        }
    }
    
    // Calculate errors (desired - actual)
    Rates error;
    error.roll = desired_rates.roll - state.rates.roll;
    error.pitch = desired_rates.pitch - state.rates.pitch;
    error.yaw = desired_rates.yaw - state.rates.yaw;
    
    LOG_DEBUG("PID errors - Roll: " + std::to_string(error.roll) + 
              ", Pitch: " + std::to_string(error.pitch) + 
              ", Yaw: " + std::to_string(error.yaw));
    
    // Calculate derivative term (if we have a previous error)
    Rates derivative;
    if (pid_state.prev_time > 0.0) {
        derivative.roll = (error.roll - pid_state.prev_error.roll) / dt;
        derivative.pitch = (error.pitch - pid_state.prev_error.pitch) / dt;
        derivative.yaw = (error.yaw - pid_state.prev_error.yaw) / dt;
    } else {
        derivative.roll = 0.0;
        derivative.pitch = 0.0;
        derivative.yaw = 0.0;
    }
    
    // Update integral term
    pid_state.integral.roll += error.roll * dt;
    pid_state.integral.pitch += error.pitch * dt;
    pid_state.integral.yaw += error.yaw * dt;
    
    // Apply anti-windup by clamping the integral term
    pid_state.integral.roll = std::max(-MAX_INTEGRAL, std::min(MAX_INTEGRAL, pid_state.integral.roll));
    pid_state.integral.pitch = std::max(-MAX_INTEGRAL, std::min(MAX_INTEGRAL, pid_state.integral.pitch));
    pid_state.integral.yaw = std::max(-MAX_INTEGRAL, std::min(MAX_INTEGRAL, pid_state.integral.yaw));
    
    // Calculate PID control output
    Rates output;
    output.roll = gains.Kp * error.roll + gains.Ki * pid_state.integral.roll + gains.Kd * derivative.roll;
    output.pitch = gains.Kp * error.pitch + gains.Ki * pid_state.integral.pitch + gains.Kd * derivative.pitch;
    output.yaw = gains.Kp * error.yaw + gains.Ki * pid_state.integral.yaw + gains.Kd * derivative.yaw;
    
    LOG_DEBUG("PID output - Roll: " + std::to_string(output.roll) + 
              ", Pitch: " + std::to_string(output.pitch) + 
              ", Yaw: " + std::to_string(output.yaw));
    
    // Store current error and time for next iteration
    pid_state.prev_error = error;
    pid_state.prev_time = current_time;
    
    return output;
}

/**
 * Reset the PID controller for a specific drone
 * @param mac_address MAC address of the drone
 */
void PIDController::reset(const std::string& mac_address) {
    LOG_INFO("Resetting PID controller for drone: " + mac_address);
    
    std::lock_guard<std::mutex> lock(mutex_);
    
    // Reset integral and previous error for the specified drone
    if (states_.find(mac_address) != states_.end()) {
        states_[mac_address].integral = Rates(0.0, 0.0, 0.0);
        states_[mac_address].prev_error = Rates(0.0, 0.0, 0.0);
        states_[mac_address].prev_time = 0.0;
    } else {
        LOG_WARNING("Cannot reset PID state for unknown drone: " + mac_address);
    }
}

/**
 * Set gains directly for a specific drone
 * @param mac_address MAC address of the drone
 * @param gains PID gains
 */
void PIDController::setGains(const std::string& mac_address, const Gains& gains) {
    LOG_INFO("Setting PID gains for drone: " + mac_address);
    
    std::lock_guard<std::mutex> lock(mutex_);
    gains_[mac_address] = gains;
}

/**
 * Get current gains for a specific drone
 * @param mac_address MAC address of the drone
 * @return PID gains
 */
PIDController::Gains PIDController::getGains(const std::string& mac_address) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (gains_.find(mac_address) != gains_.end()) {
        return gains_[mac_address];
    } else {
        LOG_WARNING("No gains found for drone: " + mac_address + ", returning defaults");
        return Gains(); // Return default gains
    }
}