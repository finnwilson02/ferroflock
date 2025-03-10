/**
 * logger.h
 * 
 * Purpose: Provides logging infrastructure for drone telemetry and command data
 * 
 * Data Flow:
 *   Input: Various telemetry data from OptiTrack, IMU readings, and drone commands
 *   Output: CSV log files with timestamped telemetry and command data
 * 
 * This module records position, orientation, IMU readings, and command data
 * to CSV files for later analysis, with each drone command tracked separately.
 */

#ifndef LOGGER_H
#define LOGGER_H

#include <string>
#include <fstream>
#include <vector>
#include <chrono>
#include <iomanip>
#include <filesystem>
#include <iostream>
#include <mutex>
#include <queue>
#include <thread>

// Define logging levels
enum LogLevel {
    LOG_LEVEL_DEBUG = 0,
    LOG_LEVEL_INFO  = 1,
    LOG_LEVEL_WARNING = 2,
    LOG_LEVEL_ERROR = 3,
    LOG_LEVEL_NONE  = 4  // Suppress all logs
};

// Global debug flag and logging level
extern bool g_debug_enabled;
extern LogLevel g_log_level;

// Logging functions for consistent output
#define LOG_INFO(msg) if (LOG_LEVEL_INFO >= g_log_level) std::cout << "[INFO] " << msg << std::endl
#define LOG_ERROR(msg) if (LOG_LEVEL_ERROR >= g_log_level) std::cerr << "[ERROR] " << msg << std::endl
#define LOG_DEBUG(msg) if (LOG_LEVEL_DEBUG >= g_log_level && g_debug_enabled) std::cout << "[DEBUG] " << msg << std::endl
#define LOG_WARNING(msg) if (LOG_LEVEL_WARNING >= g_log_level) std::cout << "[WARNING] " << msg << std::endl

// DataPoint structure to hold various types of data for logging
struct DataPoint {
    std::chrono::system_clock::time_point timestamp = std::chrono::system_clock::now();
    double x = 0.0, y = 0.0, z = 0.0;
    double qw = 0.0, qx = 0.0, qy = 0.0, qz = 0.0;
    double yaw_raw = 0.0, yaw_corrected = 0.0;
    double imu_yaw = 0.0, imu_pitch = 0.0, imu_roll = 0.0;
    double imu_agx = 0.0, imu_agy = 0.0, imu_agz = 0.0;
    std::string command_string; // Raw command string
    std::string tracker_id = "Unknown";
    
    // Default constructor
    DataPoint() = default;
    
    // Constructor with timestamp set to now
    DataPoint(const std::string& id) : timestamp(std::chrono::system_clock::now()), tracker_id(id) {}
};

class Logger {
public:
    // Constructor/Destructor
    Logger(const std::string& filename);
    ~Logger();
    
    // Open the log file
    bool open(const std::string& filename);
    
    // Close the log file
    void close();
    
    // Check if the log file is open
    bool isOpen() const;
    
    // Log a data point
    void logData(const DataPoint& data);
    
    // Log position data
    void logPosition(double x, double y, double z, double timestamp);
    
    // Log orientation data
    void logOrientation(double qw, double qx, double qy, double qz, double timestamp);
    
    // Log yaw data
    void logYaw(double raw_yaw, double corrected_yaw, double timestamp);
    
    // Log IMU data
    void logIMU(double imu_yaw, double timestamp);
    
    // Log command data
    void logCommand(const std::string& command, double timestamp);
    
    // Flush the log to disk
    void flush();
    
    // Create a unique filename with timestamp
    static std::string createUniqueFilename(const std::string& prefix, const std::string& extension);
    
    // Create a data directory if it doesn't exist
    static bool createDataDirectory(const std::string& dir_path);

private:
    std::ofstream file_;
    std::mutex file_mutex_;
    int write_count_{0};
    const int flush_interval_{10}; // Flush every 10 writes by default
    std::queue<DataPoint> data_queue_; // New queue for async data
    std::mutex queue_mutex_; // Protect the queue
    std::thread log_thread_; // Dedicated logging thread
    bool running_{false}; // Control logging thread
    void logThreadFunc(); // Thread function
};

#endif // LOGGER_H