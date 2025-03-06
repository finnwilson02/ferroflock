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

// Global debug flag
extern bool g_debug_enabled;

// Logging functions for consistent output
#define LOG_INFO(msg) std::cout << "[INFO] " << msg << std::endl
#define LOG_ERROR(msg) std::cerr << "[ERROR] " << msg << std::endl
#define LOG_DEBUG(msg) if (g_debug_enabled) std::cout << "[DEBUG] " << msg << std::endl
#define LOG_WARNING(msg) std::cout << "[WARNING] " << msg << std::endl

// DataPoint structure to hold various types of data for logging
struct DataPoint {
    std::chrono::system_clock::time_point timestamp;
    double x{0.0};
    double y{0.0};
    double z{0.0};
    double qw{1.0};
    double qx{0.0};
    double qy{0.0};
    double qz{0.0};
    double yaw_raw{0.0};
    double yaw_corrected{0.0};
    double imu_yaw{0.0};
    double commanded_yaw{0.0};
    std::string tracker_id;
    
    // Default constructor
    DataPoint() = default;
    
    // Constructor with timestamp set to now
    DataPoint(const std::string& id) : tracker_id(id), timestamp(std::chrono::system_clock::now()) {}
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
    void logCommand(double commanded_yaw, double timestamp);
    
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
};

#endif // LOGGER_H