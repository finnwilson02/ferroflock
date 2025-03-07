#include "../include/tello_imu_handler.h"
#include <iostream>
#include <thread>
#include <sstream>
#include <cstring>

// Constructor
TelloIMUHandler::TelloIMUHandler(TelloController& controller, const std::string& drone_ip)
    : controller(controller), ip(drone_ip) {
    
    // Make sure the drone exists in the controller
    if (!controller.pingDrone(ip)) {
        std::cerr << "[ERROR] Could not ping drone at " << ip << std::endl;
        return;
    }
    
    // Initialize the drone if needed
    if (!controller.sendCommand(ip, "command")) {
        std::cerr << "[ERROR] Failed to initialize drone at " << ip << std::endl;
        return;
    }
    
    // Set up the last_state_time to now
    last_state_time = std::chrono::system_clock::now();
}

// Destructor
TelloIMUHandler::~TelloIMUHandler() {
    // Stop the state receiver thread
    running = false;
    if (state_thread.joinable()) {
        state_thread.join();
    }
    
    // Close the state socket
    if (state_socket >= 0) {
        close(state_socket);
        state_socket = -1;
        socket_initialized = false;
    }
}

// Initialize drone for IMU data reception
bool TelloIMUHandler::initialize() {
    // Send 'command' to enter SDK mode
    if (!controller.sendCommand(ip, "command")) {
        std::cerr << "[ERROR] Failed to send 'command' to drone at " << ip << std::endl;
        return false;
    }
    
    // Wait for command to process
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // Disable mission pad detection to reduce processing on the drone
    if (!controller.sendCommand(ip, "moff")) {
        std::cerr << "[ERROR] Failed to disable mission pad detection on drone at " << ip << std::endl;
        return false;
    }
    
    // Initialize the state socket to receive telemetry data
    if (!initializeStateSocket()) {
        std::cerr << "[ERROR] Failed to initialize state socket for drone at " << ip << std::endl;
        return false;
    }
    
    // Start the state receiver thread
    state_thread = std::thread(&TelloIMUHandler::stateReceiver, this);
    
    std::cout << "Successfully initialized IMU data acquisition for drone at " << ip << std::endl;
    return true;
}

// Initialize the state socket (UDP port 8890)
bool TelloIMUHandler::initializeStateSocket() {
    // Close existing socket if any
    if (state_socket >= 0) {
        close(state_socket);
        state_socket = -1;
        socket_initialized = false;
    }
    
    // Create a UDP socket
    state_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (state_socket < 0) {
        std::cerr << "[ERROR] Failed to create state socket for " << ip 
                << ": " << strerror(errno) << std::endl;
        return false;
    }
    
    // Set up the socket
    int reuse = 1;
    if (setsockopt(state_socket, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse)) < 0) {
        std::cerr << "[ERROR] Failed to set SO_REUSEADDR for state socket: " 
                << strerror(errno) << std::endl;
        close(state_socket);
        return false;
    }
    
    // Set up local address structure
    struct sockaddr_in local_addr{};
    local_addr.sin_family = AF_INET;
    local_addr.sin_addr.s_addr = INADDR_ANY;
    local_addr.sin_port = htons(state_port);
    
    // Bind the socket
    if (bind(state_socket, (struct sockaddr *)&local_addr, sizeof(local_addr)) < 0) {
        std::cerr << "[ERROR] Failed to bind state socket for " << ip 
                << ": " << strerror(errno) << std::endl;
        close(state_socket);
        return false;
    }
    
    // Set socket timeout
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 100000; // 100ms timeout
    setsockopt(state_socket, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    
    socket_initialized = true;
    return true;
}

// State receiver thread function
void TelloIMUHandler::stateReceiver() {
    while (running) {
        std::optional<std::string> state_data = receiveState();
        if (state_data) {
            parseStateData(*state_data);
            {
                std::lock_guard<std::mutex> lock(data_mutex);
                last_state_time = std::chrono::system_clock::now();
            }
        }
        
        // Sleep to prevent CPU overload
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

// Parse state data
void TelloIMUHandler::parseStateData(const std::string& data) {
    std::istringstream iss(data);
    std::string token;
    std::lock_guard<std::mutex> lock(data_mutex);
    
    while (std::getline(iss, token, ';')) {
        size_t pos = token.find(':');
        if (pos != std::string::npos) {
            std::string key = token.substr(0, pos);
            std::string value = token.substr(pos + 1);
            
            try {
                if (key == "yaw") imu_yaw = std::stod(value);
                else if (key == "pitch") imu_pitch = std::stod(value);
                else if (key == "roll") imu_roll = std::stod(value);
                else if (key == "agx") imu_agx = std::stod(value);
                else if (key == "agy") imu_agy = std::stod(value);
                else if (key == "agz") imu_agz = std::stod(value);
            } catch (const std::exception& e) {
                std::cerr << "[ERROR] Parsing " << key << " failed: " << e.what() << std::endl;
            }
        }
    }
    
    // Sanity checks for accelerometer values
    if (imu_agx < -10.0 || imu_agx > 10.0 || 
        imu_agy < -10.0 || imu_agy > 10.0 || 
        imu_agz < -10.0 || imu_agz > 10.0) {
        std::cerr << "[WARNING] IMU acceleration out of range for " << ip << std::endl;
    }
}

// Receive state data from UDP port 8890
std::optional<std::string> TelloIMUHandler::receiveState(int timeout_ms) {
    if (!socket_initialized || state_socket < 0) {
        return std::nullopt;
    }
    
    char buffer[1024];
    struct sockaddr_in from_addr;
    socklen_t from_len = sizeof(from_addr);
    
    // Set socket to non-blocking
    int flags = fcntl(state_socket, F_GETFL, 0);
    fcntl(state_socket, F_SETFL, flags | O_NONBLOCK);
    
    ssize_t received = recvfrom(state_socket, buffer, sizeof(buffer) - 1, 0,
                             (struct sockaddr *)&from_addr, &from_len);
    
    // Reset blocking status
    fcntl(state_socket, F_SETFL, flags);
    
    if (received < 0) {
        if (errno != EAGAIN && errno != EWOULDBLOCK) {
            std::cerr << "[ERROR] State receive error for " << ip 
                    << ": " << strerror(errno) << std::endl;
        }
        return std::nullopt;
    }
    
    if (received == 0) {
        return std::nullopt;
    }
    
    buffer[received] = '\0';
    std::string state_data(buffer);
    
    return state_data;
}

// IMU data getters
double TelloIMUHandler::getYaw() const {
    std::lock_guard<std::mutex> lock(data_mutex);
    return imu_yaw;
}

double TelloIMUHandler::getPitch() const {
    std::lock_guard<std::mutex> lock(data_mutex);
    return imu_pitch;
}

double TelloIMUHandler::getRoll() const {
    std::lock_guard<std::mutex> lock(data_mutex);
    return imu_roll;
}

double TelloIMUHandler::getAgx() const {
    std::lock_guard<std::mutex> lock(data_mutex);
    return imu_agx;
}

double TelloIMUHandler::getAgy() const {
    std::lock_guard<std::mutex> lock(data_mutex);
    return imu_agy;
}

double TelloIMUHandler::getAgz() const {
    std::lock_guard<std::mutex> lock(data_mutex);
    return imu_agz;
}

// Check if IMU data is valid
bool TelloIMUHandler::isDataValid() const {
    std::lock_guard<std::mutex> lock(data_mutex);
    auto now = std::chrono::system_clock::now();
    auto diff = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - last_state_time).count();
    return diff < 1000; // Data is valid if updated within 1 second
}