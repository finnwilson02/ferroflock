/**
 * tello_controller.cpp
 * 
 * Purpose: Implementation of the TelloController class for drone communication
 * 
 * Data Flow:
 *   Input: Command strings from application logic
 *   Output: UDP command packets sent to drones, response data from drones
 * 
 * This implementation handles socket communication with Tello drones,
 * command formatting, transmission retries, and manages socket connections
 * for multiple drones simultaneously.
 */

#include "../include/tello_controller.h"
#include "../include/logger.h"
#include <iostream>
#include <cstring>
#include <optional>

// External reference to global logger
extern Logger* g_logger;

// Constructor
TelloController::TelloController() {
    LOG_DEBUG("TelloController initialized");
}

// Destructor
TelloController::~TelloController() {
    LOG_DEBUG("TelloController shutting down, cleaning up");
    cleanup();
}

// TelloDevice destructor
TelloController::TelloDevice::~TelloDevice() {
    if (command_socket >= 0) {
        close(command_socket);
        command_socket = -1;
    }
}

// Initialize sockets for a TelloDevice
bool TelloController::TelloDevice::initializeSockets() {
    // Close existing socket if any
    if (command_socket >= 0) {
        LOG_DEBUG("[SOCKET] Closing existing socket " + std::to_string(command_socket) + " for " + ip);
        close(command_socket);
        command_socket = -1;
        socket_valid = false;
    }

    // Create a new socket
    command_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (command_socket < 0) {
        LOG_ERROR("Failed to create socket for " + ip + ": " + strerror(errno));
        return false;
    }

    LOG_DEBUG("[SOCKET] Created new socket " + std::to_string(command_socket) 
            + " for " + ip + ":" + std::to_string(local_port));

    // Enable socket reuse
    int reuse = 1;
    if (setsockopt(command_socket, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse)) < 0) {
        LOG_ERROR("Failed to set SO_REUSEADDR for " + ip + ": " + strerror(errno));
        close(command_socket);
        command_socket = -1;
        return false;
    }

    // Set up local address
    struct sockaddr_in local_addr{};
    local_addr.sin_family = AF_INET;
    local_addr.sin_addr.s_addr = INADDR_ANY;
    local_addr.sin_port = htons(local_port);

    // Bind to local port
    if (bind(command_socket, (struct sockaddr *)&local_addr, sizeof(local_addr)) < 0) {
        LOG_ERROR("Failed to bind socket for " + ip 
                + ": " + strerror(errno));
        close(command_socket);
        command_socket = -1;
        return false;
    }

    // Set up remote address
    command_addr = {};
    command_addr.sin_family = AF_INET;
    command_addr.sin_addr.s_addr = inet_addr(ip.c_str());
    command_addr.sin_port = htons(command_port);

    // Set socket timeout
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 100000; // 100ms timeout
    setsockopt(command_socket, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    socket_valid = true;
    LOG_DEBUG("[SOCKET] Successfully initialized socket " + std::to_string(command_socket) + " for " + ip);
    return true;
}

// Send a command to a TelloDevice
bool TelloController::TelloDevice::sendCommand(const std::string& cmd) {
    LOG_DEBUG("sendCommand: Checking socket state for " + ip + ", socket: " + std::to_string(command_socket));
    if (!socket_valid || command_socket < 0) {
        LOG_ERROR("[ERROR] Invalid socket state for " + ip);
        return reinitializeAndSend(cmd);
    }
    
    std::string clean_cmd = cleanCommand(cmd);
    
    LOG_DEBUG("[SEND] IP: " + ip + " Port: " + std::to_string(local_port) + 
            " Socket: " + std::to_string(command_socket) + 
            " Command: " + clean_cmd);
    
    ssize_t sent = sendto(command_socket, clean_cmd.c_str(), clean_cmd.length(), 0,
                        (struct sockaddr *)&command_addr, sizeof(command_addr));
    
    if (sent < 0) {
        LOG_ERROR("Send failed for " + ip + ": " + strerror(errno) + ", socket: " + std::to_string(command_socket));
        socket_valid = false;
        return reinitializeAndSend(clean_cmd);
    }
    
    LOG_DEBUG("[SUCCESS] Sent " + std::to_string(sent) + " bytes to " + ip);
    LOG_DEBUG("[DEBUG] Command '" + clean_cmd + "' sent to " + ip + ", bytes: " + std::to_string(sent));
    
    // Log the raw command string if logger is available
    if (g_logger && g_logger->isOpen()) {
        static std::mutex logger_mutex;
        std::lock_guard<std::mutex> lock(logger_mutex);
        g_logger->logCommand(clean_cmd, std::chrono::system_clock::to_time_t(std::chrono::system_clock::now()));
    }
    
    // Don't wait for responses from Tellos - they're unreliable
    // Just assume success if the send worked
    
    return true;
}

// Reinitialize sockets and send command
bool TelloController::TelloDevice::reinitializeAndSend(const std::string& cmd) {
    if (initializeSockets()) {
        return sendCommand(cmd);
    }
    return false;
}

// Receive a response from a TelloDevice
std::optional<std::string> TelloController::TelloDevice::receiveResponse() {
    if (!socket_valid || command_socket < 0) return std::nullopt;
    
    // Use non-blocking receive to avoid hanging
    char buffer[1024];
    struct sockaddr_in from_addr;
    socklen_t from_len = sizeof(from_addr);
    
    // Set socket to non-blocking for this receive
    int flags = fcntl(command_socket, F_GETFL, 0);
    fcntl(command_socket, F_SETFL, flags | O_NONBLOCK);
    
    ssize_t received = recvfrom(command_socket, buffer, sizeof(buffer)-1, 0,
                            (struct sockaddr *)&from_addr, &from_len);
    
    // Reset blocking status
    fcntl(command_socket, F_SETFL, flags);
    
    if (received < 0) {
        if (errno != EAGAIN && errno != EWOULDBLOCK) {
            LOG_ERROR("Receive error for " + ip + ": " 
                    + strerror(errno));
        }
        return std::nullopt;
    }
    
    if (received == 0) return std::nullopt;
    
    buffer[received] = '\0';
    std::string response(buffer);
    
    LOG_DEBUG("[RESPONSE] IP: " + ip + " Response: " + response);
    return response;
}

// Clean command to remove invalid characters
std::string TelloController::TelloDevice::cleanCommand(const std::string& cmd) {
    std::string clean;
    for(char c : cmd) {
        if (isalnum(c) || c == ' ' || c == '?' || c == '-') {
            clean += c;
        }
    }
    return clean;
}

// Initialize a drone with its IP address
bool TelloController::initialize(const std::string& ip, bool skip_reboot) {
    // Check if already initialized
    if (devices.find(ip) != devices.end()) {
        if (!devices[ip].socket_valid) {
            LOG_WARNING("Reinitializing invalid socket for " + ip);
            devices[ip].initializeSockets(); // Reinitialize if invalid
        }
        
        // If device is already initialized but needs reboot and we're not skipping it
        if (devices[ip].needs_reboot && !skip_reboot) {
            LOG_DEBUG("Rebooting drone at " + ip + " before use...");
            rebootDrone(ip);
            devices[ip].needs_reboot = false;
            
            // Wait for reboot to complete
            LOG_DEBUG("Waiting for drone to reboot (15 seconds)...");
            std::this_thread::sleep_for(std::chrono::seconds(15));
            
            // Reconnect sockets after reboot
            if (!devices[ip].initializeSockets()) {
                LOG_ERROR("Failed to reinitialize sockets after reboot");
                return false;
            }
        }
        return true;
    }
    
    // Validate IP
    if (ip.find('.') == std::string::npos) {
        LOG_ERROR("Invalid IP address: " + ip);
        return false;
    }
    
    // Skip rebooting at the start of routine
    LOG_DEBUG("Initializing drone at " + ip + " (no reboot)");
    
    TelloDevice device(ip, 8889, next_port++);
    device.needs_reboot = false; // No need to reboot
    
    if (!device.initializeSockets()) {
        LOG_ERROR("Failed to initialize sockets for " + ip);
        return false;
    }
    
    // Enter SDK mode - send the command multiple times since Tellos are unreliable
    LOG_DEBUG("Sending SDK mode command to " + ip);
    
    // Send command 3 times for redundancy
    for (int i = 0; i < 3; i++) {
        device.sendCommand("command");
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    // Don't wait for response - assume it worked
    LOG_INFO("Initialized TelloDevice for " + ip + " with socket " + std::to_string(device.command_socket));
    devices[ip] = std::move(device);
    return true;
}

// Send a command to a drone
bool TelloController::sendCommand(const std::string& ip, const std::string& command, bool skip_reboot) {
    if (devices.find(ip) == devices.end()) {
        if (!initialize(ip, skip_reboot)) {
            LOG_ERROR("Failed to initialize device at " + ip + " for command: " + command);
            return false;
        }
    }
    
    bool result = devices[ip].sendCommand(command);
    if (!result) {
        LOG_ERROR("Failed to send command '" + command + "' to " + ip);
    }
    return result;
}

// Receive a response from a drone
std::optional<std::string> TelloController::receiveResponse(const std::string& ip, int timeout_ms) {
    if (devices.find(ip) == devices.end()) {
        return std::nullopt;
    }
    
    // For land commands, don't wait for response
    // Try once then return quickly to avoid hanging
    auto response = devices[ip].receiveResponse();
    if (response) {
        return response;
    }
    
    // For other commands, try a few times but with a short timeout
    if (timeout_ms > 0) {
        auto start = std::chrono::steady_clock::now();
        while (std::chrono::steady_clock::now() - start < std::chrono::milliseconds(timeout_ms)) {
            response = devices[ip].receiveResponse();
            if (response) {
                return response;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
    
    return std::nullopt;
}

// Reboot a specific drone
bool TelloController::rebootDrone(const std::string& ip, int port) {
    LOG_DEBUG("Rebooting drone at " + ip + "...");
    
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) {
        LOG_ERROR("Failed to create socket for reboot: " + std::string(strerror(errno)));
        return false;
    }

    // Set up address
    struct sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = inet_addr(ip.c_str());
    addr.sin_port = htons(port);

    // Send reboot command
    std::string cmd = "reboot";
    ssize_t sent = sendto(sock, cmd.c_str(), cmd.length(), 0, 
                      (struct sockaddr*)&addr, sizeof(addr));
    
    close(sock);
    
    if (sent < 0) {
        LOG_ERROR("Failed to send reboot to " + ip + ": " + std::string(strerror(errno)));
        return false;
    }
    
    LOG_DEBUG("Sent reboot command to " + ip);
    return true;
}

// Check if a drone is responding to ping
bool TelloController::pingDrone(const std::string& ip) {
    std::string cmd = "ping -c 1 -W 1 " + ip + " > /dev/null 2>&1";
    return system(cmd.c_str()) == 0;
}

// Perform cleanup
void TelloController::cleanup() {
    for (auto& [ip, device] : devices) {
        if (device.socket_valid) {
            // Try to land first, then reboot
            LOG_DEBUG("Attempting to land drone at " + ip);
            for (int i = 0; i < 3; i++) {
                device.sendCommand("land");
                std::this_thread::sleep_for(std::chrono::milliseconds(300));
            }
            
            // Wait for landing to complete
            std::this_thread::sleep_for(std::chrono::seconds(3));
            
            // Then reboot for clean state
            LOG_DEBUG("Rebooting drone at " + ip + " for clean exit");
            rebootDrone(ip);
        }
    }
    
    // Close all sockets and clear the devices map
    for (auto& [ip, device] : devices) {
        if (device.command_socket >= 0) {
            close(device.command_socket);
            device.command_socket = -1;
            device.socket_valid = false;
        }
    }
    devices.clear();
    
    LOG_DEBUG("All drones have been landed and rebooted");
}

// Send a command to all drones simultaneously
void TelloController::sendCommandToAll(const std::string& command) {
    for (auto& [ip, device] : devices) {
        device.sendCommand(command);
    }
}