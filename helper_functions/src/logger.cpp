/**
 * logger.cpp
 * 
 * Purpose: Implementation of the Logger class for tracking drone telemetry and commands
 * 
 * Data Flow:
 *   Input: Position, orientation, IMU, and command data from various system components
 *   Output: CSV log files containing timestamped telemetry and command data
 *          (stored in /home/finn/ferroflock/helper_functions/data/)
 * 
 * This module implements CSV-based logging with support for tracking all drone command types
 * separately, facilitating detailed post-flight analysis.
 */

#include "../include/logger.h"

// Define global logging variables (these are already defined in main.cpp)
LogLevel g_log_level; // Defined in main.cpp
// bool g_debug_enabled is defined in main.cpp

// Constructor
Logger::Logger(const std::string& filename) {
    LOG_INFO("Initializing logger with file: " + filename);
    
    // Extract the base filename without the directory path
    std::string base_filename = filename;
    size_t pos = filename.find_last_of("/\\");
    if (pos != std::string::npos) {
        base_filename = filename.substr(pos + 1);
    }
    
    std::string full_filename = "/home/finn/ferroflock/helper_functions/data/" + base_filename;
    LOG_DEBUG("Full path for log file: " + full_filename);
    if (open(full_filename)) {
        running_ = true;
        log_thread_ = std::thread(&Logger::logThreadFunc, this);
    }
}

// Destructor
Logger::~Logger() {
    LOG_INFO("Closing logger");
    running_ = false;
    if (log_thread_.joinable()) {
        log_thread_.join();
    }
    flush(); // Ensure all data is written
    close();
}

// Open the log file
bool Logger::open(const std::string& filename) {
    std::lock_guard<std::mutex> lock(file_mutex_);
    
    if (file_.is_open()) {
        LOG_WARNING("Log file already open, closing before opening new file");
        file_.close();
    }
    
    file_.open(filename, std::ios::app);
    if (!file_.is_open()) {
        LOG_ERROR("Failed to open log file: " + filename);
        return false;
    }
    
    LOG_INFO("Successfully opened log file: " + filename);
    
    // Write header if file is new
    if (file_.tellp() == 0) {
        LOG_DEBUG("Adding CSV header to log file");
        file_ << "timestamp,x,y,z,qw,qx,qy,qz,yaw_raw,yaw_corrected,"
              << "imu_yaw,imu_pitch,imu_roll,imu_agx,imu_agy,imu_agz,"
              << "command_string,tracker_id\n";
        file_.flush();
    }
    
    return true;
}

// Close the log file
void Logger::close() {
    std::lock_guard<std::mutex> lock(file_mutex_);
    
    if (file_.is_open()) {
        LOG_DEBUG("Flushing and closing log file");
        file_.flush();
        file_.close();
    }
}

// Check if the log file is open
bool Logger::isOpen() const {
    return file_.is_open();
}

// Log a data point
void Logger::logData(const DataPoint& data) {
    LOG_DEBUG("logData: Adding data to queue");
    std::lock_guard<std::mutex> lock(queue_mutex_);
    data_queue_.push(data);
}

// Log position data
void Logger::logPosition(double x, double y, double z, double timestamp) {
    DataPoint data;
    data.timestamp = std::chrono::system_clock::from_time_t(static_cast<time_t>(timestamp));
    data.x = x;
    data.y = y;
    data.z = z;
    logData(data);
}

// Log orientation data
void Logger::logOrientation(double qw, double qx, double qy, double qz, double timestamp) {
    DataPoint data;
    data.timestamp = std::chrono::system_clock::from_time_t(static_cast<time_t>(timestamp));
    data.qw = qw;
    data.qx = qx;
    data.qy = qy;
    data.qz = qz;
    logData(data);
}

// Log yaw data
void Logger::logYaw(double raw_yaw, double corrected_yaw, double timestamp) {
    DataPoint data;
    data.timestamp = std::chrono::system_clock::from_time_t(static_cast<time_t>(timestamp));
    data.yaw_raw = raw_yaw;
    data.yaw_corrected = corrected_yaw;
    logData(data);
}

// Log IMU data
void Logger::logIMU(double imu_yaw, double timestamp) {
    DataPoint data;
    data.timestamp = std::chrono::system_clock::from_time_t(static_cast<time_t>(timestamp));
    data.imu_yaw = imu_yaw;
    logData(data);
}

// Log command data
void Logger::logCommand(const std::string& command, double timestamp) {
    DataPoint data;
    data.timestamp = std::chrono::system_clock::from_time_t(static_cast<time_t>(timestamp));
    data.command_string = command;
    logData(data);
}

// Flush the log to disk
void Logger::flush() {
    std::lock_guard<std::mutex> lock(file_mutex_);
    
    // Process any remaining data in the queue
    while (true) {
        DataPoint data;
        bool has_data = false;
        {
            std::lock_guard<std::mutex> q_lock(queue_mutex_);
            if (!data_queue_.empty()) {
                data = data_queue_.front();
                data_queue_.pop();
                has_data = true;
            } else {
                break;
            }
        }
        
        if (has_data && file_.is_open()) {
            auto time_t = std::chrono::system_clock::to_time_t(data.timestamp);
            std::stringstream ss;
            ss << std::put_time(std::localtime(&time_t), "%Y-%m-%d %H:%M:%S");
            file_ << ss.str() << ","
                  << data.x << "," << data.y << "," << data.z << ","
                  << data.qw << "," << data.qx << "," << data.qy << "," << data.qz << ","
                  << data.yaw_raw << "," << data.yaw_corrected << ","
                  << data.imu_yaw << "," << data.imu_pitch << "," << data.imu_roll << ","
                  << data.imu_agx << "," << data.imu_agy << "," << data.imu_agz << ","
                  << "\"" << data.command_string << "\"," << data.tracker_id << "\n";
        }
    }
    
    if (file_.is_open()) {
        LOG_DEBUG("Manually flushing log file");
        file_.flush();
        
        // Check for file errors after flush
        if (file_.fail()) {
            LOG_ERROR("Error detected after manually flushing log file");
            file_.clear(); // Clear error flags to continue writing
        }
    } else {
        LOG_WARNING("Cannot flush - log file is not open");
    }
}

// Create a unique filename with timestamp
std::string Logger::createUniqueFilename(const std::string& prefix, const std::string& extension) {
    auto now = std::chrono::system_clock::now();
    auto now_time_t = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << prefix << "_" << std::put_time(std::localtime(&now_time_t), "%Y%m%d_%H%M%S") << extension;
    
    return ss.str();
}

// Create a data directory if it doesn't exist
bool Logger::createDataDirectory(const std::string& dir_path) {
    std::string full_path = "/home/finn/ferroflock/helper_functions/data";
    try {
        LOG_DEBUG("Creating directory: " + full_path);
        std::filesystem::create_directories(full_path);
        LOG_DEBUG("Directory created/verified successfully");
        return true;
    } catch (const std::exception& e) {
        LOG_ERROR("Failed to create directory: " + std::string(e.what()));
        return false;
    }
}

// Thread function for logging data
void Logger::logThreadFunc() {
    LOG_DEBUG("Logging thread started");
    
    while (running_ || !data_queue_.empty()) {
        DataPoint data;
        bool has_data = false;
        
        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            if (!data_queue_.empty()) {
                data = data_queue_.front();
                data_queue_.pop();
                has_data = true;
            }
        }
        
        if (has_data) {
            std::lock_guard<std::mutex> lock(file_mutex_);
            if (file_.is_open()) {
                auto time_t = std::chrono::system_clock::to_time_t(data.timestamp);
                std::stringstream ss;
                ss << std::put_time(std::localtime(&time_t), "%Y-%m-%d %H:%M:%S");
                file_ << ss.str() << ","
                      << data.x << "," << data.y << "," << data.z << ","
                      << data.qw << "," << data.qx << "," << data.qy << "," << data.qz << ","
                      << data.yaw_raw << "," << data.yaw_corrected << ","
                      << data.imu_yaw << "," << data.imu_pitch << "," << data.imu_roll << ","
                      << data.imu_agx << "," << data.imu_agy << "," << data.imu_agz << ","
                      << "\"" << data.command_string << "\"," << data.tracker_id << "\n";
                
                LOG_DEBUG("Logged data point, queue size: " + std::to_string(data_queue_.size()));
                if (data_queue_.size() > 1000) { // Arbitrary threshold
                    LOG_WARNING("Data queue size exceeded 1000, possible data loss");
                }
                
                if (++write_count_ % flush_interval_ == 0) {
                    file_.flush();
                    if (file_.fail()) {
                        LOG_ERROR("Error detected after flushing log file");
                        file_.clear();
                    }
                }
            }
        }
        
        // Sleep for 5ms (200Hz writing rate)
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    
    LOG_DEBUG("Logging thread stopped");
}