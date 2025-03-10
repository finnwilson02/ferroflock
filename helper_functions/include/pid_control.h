/**
 * pid_control.h
 * 
 * Purpose: Provides PID (Proportional-Integral-Derivative) control for drone flight
 * 
 * Data Flow:
 *   Input: Desired rates and estimated rates from UKF state vector
 *   Output: Control commands to stabilize the drone
 * 
 * This module implements a PID controller that calculates control outputs based on
 * the difference between desired rates and estimated rates. It supports separate
 * gains for different drones, identified by their MAC addresses.
 */

#ifndef PID_CONTROL_H
#define PID_CONTROL_H

#include <string>
#include <map>
#include <mutex>
#include <vector>
#include <nlohmann/json.hpp>
#include "logger.h"

class PIDController {
public:
    /**
     * Struct to hold roll, pitch, and yaw rates
     */
    struct Rates {
        double roll{0.0};    // Roll rate in radians/second
        double pitch{0.0};   // Pitch rate in radians/second
        double yaw{0.0};     // Yaw rate in radians/second
        
        Rates() = default;
        
        Rates(double r, double p, double y) 
            : roll(r), pitch(p), yaw(y) {}
    };
    
    /**
     * Struct to hold PID gains for a drone
     */
    struct Gains {
        double Kp{1.0};  // Proportional gain (default: 1.0)
        double Ki{0.0};  // Integral gain (default: 0.0)
        double Kd{0.0};  // Derivative gain (default: 0.0)
        
        Gains() = default;
        
        Gains(double kp, double ki, double kd)
            : Kp(kp), Ki(ki), Kd(kd) {}
    };
    
    /**
     * Struct to hold PID state for each axis (roll, pitch, yaw)
     */
    struct PIDState {
        Rates integral{0.0, 0.0, 0.0};      // Accumulated error
        Rates prev_error{0.0, 0.0, 0.0};    // Previous error
        double prev_time{0.0};              // Previous update time
    };
    
    /**
     * Struct for the UKF state vector
     * Currently we only use the rates, but this can be extended
     */
    struct StateVector {
        Rates rates;      // Estimated rates from UKF
        // Can be extended with other state variables as needed
    };

public:
    /**
     * Constructor
     */
    PIDController();
    
    /**
     * Destructor
     */
    virtual ~PIDController() = default;
    
    /**
     * Initialize the PID controller for a specific drone
     * @param mac_address MAC address of the drone
     * @return True if initialization succeeded, false otherwise
     */
    bool initialize(const std::string& mac_address);
    
    /**
     * Load PID gains from JSON file
     * @param filename Path to JSON file, defaults to standard locations
     * @return True if loading succeeded, false otherwise
     */
    bool loadGainsFromJSON(const std::string& filename = "");
    
    /**
     * Compute control output based on desired rates and current state
     * @param desired_rates Desired roll, pitch, and yaw rates
     * @param state Current state vector from UKF
     * @param current_time Current time in seconds
     * @return Control commands as roll, pitch, and yaw rates
     */
    Rates computeControl(const Rates& desired_rates, 
                        const StateVector& state,
                        double current_time);
    
    /**
     * Reset the PID controller for a specific drone
     * @param mac_address MAC address of the drone
     */
    void reset(const std::string& mac_address);
    
    /**
     * Set gains directly for a specific drone
     * @param mac_address MAC address of the drone
     * @param gains PID gains
     */
    void setGains(const std::string& mac_address, const Gains& gains);
    
    /**
     * Get current gains for a specific drone
     * @param mac_address MAC address of the drone
     * @return PID gains
     */
    Gains getGains(const std::string& mac_address);

private:
    // MAC address to PID gains mapping
    std::map<std::string, Gains> gains_;
    
    // MAC address to PID state mapping
    std::map<std::string, PIDState> states_;
    
    // Currently active MAC address
    std::string active_mac_;
    
    // Mutex for thread safety
    std::mutex mutex_;
    
    // Default time step if none provided (in seconds)
    static constexpr double DEFAULT_DT = 0.01; // 100 Hz
    
    // Maximum allowed integral term to prevent windup
    static constexpr double MAX_INTEGRAL = 1.0;
};

#endif // PID_CONTROL_H