#include "../include/logger.h"

// Constructor
Logger::Logger(const std::string& filename) {
    LOG_INFO("Initializing logger with file: " + filename);
    open(filename);
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
        file_ << "timestamp,x,y,z,qw,qx,qy,qz,yaw_raw,yaw_corrected,imu_yaw,commanded_yaw,tracker_id\n";
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
          << data.commanded_yaw << ","
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
void Logger::logCommand(double commanded_yaw, double timestamp) {
    DataPoint data;
    data.timestamp = std::chrono::system_clock::from_time_t(static_cast<time_t>(timestamp));
    data.commanded_yaw = commanded_yaw;
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
    try {
        LOG_DEBUG("Creating directory: " + dir_path);
        std::filesystem::create_directories(dir_path);
        LOG_DEBUG("Directory created/verified successfully");
        return true;
    } catch (const std::exception& e) {
        LOG_ERROR("Failed to create directory: " + std::string(e.what()));
        return false;
    }
}