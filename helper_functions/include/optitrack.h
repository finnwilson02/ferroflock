#ifndef OPTITRACK_H
#define OPTITRACK_H

#include <string>
#include <map>
#include <mutex>
#include <thread>
#include <vector>
#include <chrono>
#include <functional>
#include <opencv2/opencv.hpp>
#include <vrpn_Tracker.h>

// Structure to hold position and orientation data from OptiTrack
struct TrackerData {
    double x{0}, y{0}, z{0};
    double qx{0}, qy{0}, qz{0}, qw{1.0}; // Quaternion for orientation
    cv::Scalar color;
    bool updated{false};
    std::chrono::system_clock::time_point last_update;
    std::vector<cv::Point> path;  // Store path history
    static const size_t MAX_PATH_LENGTH = 2500;  // Maximum number of points to store
    
    // Yaw correction data
    double last_raw_yaw{0.0};       // Last uncorrected yaw from quaternion
    double last_corrected_yaw{0.0}; // Last corrected yaw after flip detection
    bool has_prior_yaw{false};      // Flag to indicate if we have a prior yaw measurement
    
    // Logging data for yaw correction analysis
    std::vector<double> raw_yaw_log;
    std::vector<double> corrected_yaw_log;
    std::vector<std::chrono::system_clock::time_point> yaw_log_timestamps;
};

// DroneData structure to combine tracker and drone info
struct DroneData {
    std::string name;        // Human-readable name (e.g., "Bird 3")
    std::string tracker_id;  // OptiTrack tracker ID (e.g., "Tracker4")
    std::string ip;          // IP address of the drone (e.g., "192.168.1.108")
    double yaw_offset{0.0};  // Calibrated yaw offset
};

class OptiTrack {
public:
    // Constructor/Destructor
    OptiTrack();
    ~OptiTrack();
    
    // Set the global instance for VRPN callbacks
    static void setInstance(OptiTrack* instance) {
        global_instance_ = instance;
    }
    
    // Get the global instance for VRPN callbacks
    static OptiTrack* getInstance() {
        return global_instance_;
    }
    
    // Initialize OptiTrack system
    void initialize();
    
    // Start visualization in a separate thread
    void startVisualization();
    
    // Stop visualization
    void stopVisualization();
    
    // Update visualization with current position and orientation
    void updateVisualization(double x, double y, double yaw);
    
    // Get X position for a specific tracker
    double getXPosition(const std::string& tracker_name);
    
    // Get Y position for a specific tracker
    double getYPosition(const std::string& tracker_name);
    
    // Get Z position for a specific tracker
    double getZPosition(const std::string& tracker_name);
    
    // Get yaw for a specific tracker
    double getYaw(const std::string& tracker_name);
    
    // Get the raw (uncorrected) yaw for a specific tracker
    double getRawYaw(const std::string& tracker_name);
    
    // Set up drone-to-tracker mapping
    void setupDroneMapping(const std::vector<DroneData>& drones);
    
    // Get list of all active trackers
    std::vector<std::string> getActiveTrackers();
    
    // Check if a tracker is active/visible
    bool isTrackerActive(const std::string& tracker_name);
    
    // Convert world coordinates to screen coordinates for visualization
    cv::Point worldToScreen(double x, double y, const cv::Point& center);
    
    // Convert quaternion to yaw angle
    double quaternionToYaw(double qw, double qx, double qy, double qz);
    
    // Correct yaw flip (handling OptiTrack's 180° ambiguity)
    double correctYawFlip(double measured_yaw, double previous_yaw);

private:
    // Map of tracker names to TrackerData objects
    std::map<std::string, TrackerData> trackers_;
    
    // Mutex for thread safety
    std::mutex tracker_mutex_;
    
    // VRPN trackers
    std::vector<std::pair<std::string*, vrpn_Tracker_Remote*>> vrpn_trackers_;
    
    // Visualization settings
    const double scale_{50.0};    // Scale factor for visualization
    const double offset_x_{0.0};  // Offset to center the visualization
    const double offset_y_{0.0};  // Offset to center the visualization
    
    // Thread for visualization
    std::thread viz_thread_;
    bool running_{false};
    bool viz_initialized_{false};
    
    // Drone mapping
    std::vector<DroneData> drones_;
    
    // VRPN callback function
    static void VRPN_CALLBACK handleTrackerData(void* userData, const vrpn_TRACKERCB t);
    
    // Global instance for VRPN callbacks
    static OptiTrack* global_instance_;
    
    // Visualization thread function
    void visualizationThreadFunc();
    
    // Initialize trackers
    void initializeTrackers();
};

#endif // OPTITRACK_H