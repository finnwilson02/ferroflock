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

// Define global logging level
LogLevel g_log_level = LOG_LEVEL_DEBUG; // Default to showing all messages

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
    open(full_filename);
}

// Destructor
Logger::~Logger() {
    LOG_INFO("Closing logger");
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
              << "command_command,command_takeoff,command_land,command_up,command_down,"
              << "command_left,command_right,command_forward,command_back,command_cw,"
              << "command_ccw,command_flip,command_speed,command_reboot,command_stop,"
              << "command_emergency,tracker_id\n";
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
    std::lock_guard<std::mutex> lock(file_mutex_);
    
    if (!file_.is_open()) {
        LOG_ERROR("Cannot log data - log file is not open");
        return;
    }
    
    // Convert timestamp to string
    auto time_t = std::chrono::system_clock::to_time_t(data.timestamp);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&time_t), "%Y-%m-%d %H:%M:%S");
    
    // Write data
    file_ << ss.str() << ","
          << data.x << ","
          << data.y << ","
          << data.z << ","
          << data.qw << ","
          << data.qx << ","
          << data.qy << ","
          << data.qz << ","
          << data.yaw_raw << ","
          << data.yaw_corrected << ","
          << data.imu_yaw << ","
          << data.imu_pitch << ","
          << data.imu_roll << ","
          << data.imu_agx << ","
          << data.imu_agy << ","
          << data.imu_agz << ","
          << data.command_command << ","
          << data.command_takeoff << ","
          << data.command_land << ","
          << data.command_up << ","
          << data.command_down << ","
          << data.command_left << ","
          << data.command_right << ","
          << data.command_forward << ","
          << data.command_back << ","
          << data.command_cw << ","
          << data.command_ccw << ","
          << data.command_flip << ","
          << data.command_speed << ","
          << data.command_reboot << ","
          << data.command_stop << ","
          << data.command_emergency << ","
          << data.tracker_id << "\n";
    
    // Flush every flush_interval_ writes
    if (++write_count_ % flush_interval_ == 0) {
        LOG_DEBUG("Flushing log file after " + std::to_string(write_count_) + " writes");
        file_.flush();
        
        // Check for file errors after flush
        if (file_.fail()) {
            LOG_ERROR("Error detected after flushing log file");
            file_.clear(); // Clear error flags to continue writing
        }
    }
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
void Logger::logCommand(const std::string& command, double value, double timestamp) {
    DataPoint data;
    data.timestamp = std::chrono::system_clock::from_time_t(static_cast<time_t>(timestamp));
    
    // Set all command fields to -1.0 by default
    data.command_command = -1.0;
    data.command_takeoff = -1.0;
    data.command_land = -1.0;
    data.command_up = -1.0;
    data.command_down = -1.0;
    data.command_left = -1.0;
    data.command_right = -1.0;
    data.command_forward = -1.0;
    data.command_back = -1.0;
    data.command_cw = -1.0;
    data.command_ccw = -1.0;
    data.command_flip = -1.0;
    data.command_speed = -1.0;
    data.command_reboot = -1.0;
    data.command_stop = -1.0;
    data.command_emergency = -1.0;

    // Set the specific command field based on the input
    if (command == "command") {
        data.command_command = 1.0; // Presence indicator
    } else if (command == "takeoff") {
        data.command_takeoff = 1.0;
    } else if (command == "land") {
        data.command_land = 1.0;
    } else if (command == "up") {
        data.command_up = value;
    } else if (command == "down") {
        data.command_down = value;
    } else if (command == "left") {
        data.command_left = value;
    } else if (command == "right") {
        data.command_right = value;
    } else if (command == "forward") {
        data.command_forward = value;
    } else if (command == "back") {
        data.command_back = value;
    } else if (command == "cw") {
        data.command_cw = value;
    } else if (command == "ccw") {
        data.command_ccw = value;
    } else if (command == "flip") {
        data.command_flip = value; // 0-3 for direction
    } else if (command == "speed") {
        data.command_speed = value;
    } else if (command == "reboot") {
        data.command_reboot = 1.0;
    } else if (command == "stop") {
        data.command_stop = 1.0;
    } else if (command == "emergency") {
        data.command_emergency = 1.0;
    } else {
        LOG_WARNING("Unknown command '" + command + "' - logging with all commands as -1.0");
    }
    
    logData(data);
}

// Flush the log to disk
void Logger::flush() {
    std::lock_guard<std::mutex> lock(file_mutex_);
    
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