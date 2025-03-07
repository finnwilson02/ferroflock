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
    std::cout << "TelloController initialized" << std::endl;
}

// Destructor
TelloController::~TelloController() {
    std::cout << "TelloController shutting down, cleaning up" << std::endl;
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
        close(command_socket);
        command_socket = -1;
        socket_valid = false;
    }

    command_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (command_socket < 0) {
        std::cerr << "Failed to create socket for " << ip << ": " << strerror(errno) << std::endl;
        return false;
    }

    // Duplicate the socket to ensure unique descriptor
    int new_socket = dup(command_socket);
    close(command_socket);
    command_socket = new_socket;

    std::cout << "[SOCKET] Created new socket " << command_socket 
            << " for " << ip << ":" << local_port << std::endl;

    // Enable socket reuse
    int reuse = 1;
    if (setsockopt(command_socket, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse)) < 0) {
        std::cerr << "Failed to set SO_REUSEADDR for " << ip << ": " << strerror(errno) << std::endl;
        close(command_socket);
        return false;
    }

    // Set up local address
    struct sockaddr_in local_addr{};
    local_addr.sin_family = AF_INET;
    local_addr.sin_addr.s_addr = INADDR_ANY;
    local_addr.sin_port = htons(local_port);

    // Bind to local port
    if (bind(command_socket, (struct sockaddr *)&local_addr, sizeof(local_addr)) < 0) {
        std::cerr << "Failed to bind socket for " << ip 
                << ": " << strerror(errno) << std::endl;
        close(command_socket);
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
    return true;
}

// Send a command to a TelloDevice
bool TelloController::TelloDevice::sendCommand(const std::string& cmd) {
    if (!socket_valid || command_socket < 0) {
        std::cerr << "[ERROR] Invalid socket state for " << ip << std::endl;
        return reinitializeAndSend(cmd);
    }
    
    std::string clean_cmd = cleanCommand(cmd);
    
    std::cout << "[SEND] IP: " << ip << " Port: " << local_port 
            << " Socket: " << command_socket 
            << " Command: " << clean_cmd << std::endl;
    
    ssize_t sent = sendto(command_socket, clean_cmd.c_str(), clean_cmd.length(), 0,
                        (struct sockaddr *)&command_addr, sizeof(command_addr));
    
    if (sent < 0) {
        std::cerr << "Send failed for " << ip << ": " << strerror(errno) << std::endl;
        socket_valid = false;
        return reinitializeAndSend(clean_cmd);
    }
    
    std::cout << "[SUCCESS] Sent " << sent << " bytes to " << ip << std::endl;
    std::cout << "[DEBUG] Command '" << clean_cmd << "' sent to " << ip << ", bytes: " << sent << std::endl;
    
    // Log the command using global logger if available
    if (g_logger && g_logger->isOpen()) {
        std::string command_name = clean_cmd;
        double value = 1.0; // Default for commands without values
        size_t space_pos = clean_cmd.find(' ');
        if (space_pos != std::string::npos) {
            command_name = clean_cmd.substr(0, space_pos);
            try {
                value = std::stod(clean_cmd.substr(space_pos + 1));
            } catch (const std::exception&) {
                value = 1.0; // Fallback if parsing fails
            }
        }
        g_logger->logCommand(command_name, value, std::chrono::system_clock::to_time_t(std::chrono::system_clock::now()));
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
            std::cerr << "Receive error for " << ip << ": " 
                    << strerror(errno) << std::endl;
        }
        return std::nullopt;
    }
    
    if (received == 0) return std::nullopt;
    
    buffer[received] = '\0';
    std::string response(buffer);
    
    std::cout << "[RESPONSE] IP: " << ip << " Response: " << response << std::endl;
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
        // If device is already initialized but needs reboot and we're not skipping it
        if (devices[ip].needs_reboot && !skip_reboot) {
            std::cout << "Rebooting drone at " << ip << " before use..." << std::endl;
            rebootDrone(ip);
            devices[ip].needs_reboot = false;
            
            // Wait for reboot to complete
            std::cout << "Waiting for drone to reboot (15 seconds)..." << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(15));
            
            // Reconnect sockets after reboot
            if (!devices[ip].initializeSockets()) {
                std::cerr << "Failed to reinitialize sockets after reboot" << std::endl;
                return false;
            }
        }
        return true;
    }
    
    // Skip rebooting at the start of routine
    std::cout << "Initializing drone at " << ip << " (no reboot)" << std::endl;
    
    TelloDevice device;
    device.ip = ip;
    device.local_port = next_port++;
    device.needs_reboot = false; // No need to reboot
    
    if (!device.initializeSockets()) {
        std::cerr << "Failed to initialize sockets for " << ip << std::endl;
        return false;
    }
    
    // Enter SDK mode - send the command multiple times since Tellos are unreliable
    std::cout << "Sending SDK mode command to " << ip << std::endl;
    
    // Send command 3 times for redundancy
    for (int i = 0; i < 3; i++) {
        device.sendCommand("command");
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    // Don't wait for response - assume it worked
    std::cout << "Assuming drone at " << ip << " entered SDK mode" << std::endl;
    devices[ip] = std::move(device);
    return true;
}

// Send a command to a drone
bool TelloController::sendCommand(const std::string& ip, const std::string& command, bool skip_reboot) {
    if (devices.find(ip) == devices.end()) {
        if (!initialize(ip, skip_reboot)) {
            std::cerr << "Failed to initialize device at " << ip << " for command: " << command << std::endl;
            return false;
        }
    }
    
    bool result = devices[ip].sendCommand(command);
    if (!result) {
        std::cerr << "Failed to send command '" << command << "' to " << ip << std::endl;
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
    std::cout << "Rebooting drone at " << ip << "..." << std::endl;
    
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) {
        std::cerr << "Failed to create socket for reboot: " << strerror(errno) << std::endl;
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
        std::cerr << "Failed to send reboot to " << ip << ": " << strerror(errno) << std::endl;
        return false;
    }
    
    std::cout << "Sent reboot command to " << ip << std::endl;
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
            std::cout << "Attempting to land drone at " << ip << std::endl;
            for (int i = 0; i < 3; i++) {
                device.sendCommand("land");
                std::this_thread::sleep_for(std::chrono::milliseconds(300));
            }
            
            // Wait for landing to complete
            std::this_thread::sleep_for(std::chrono::seconds(3));
            
            // Then reboot for clean state
            std::cout << "Rebooting drone at " << ip << " for clean exit" << std::endl;
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
    
    std::cout << "All drones have been landed and rebooted" << std::endl;
}

// Send a command to all drones simultaneously
void TelloController::sendCommandToAll(const std::string& command) {
    for (auto& [ip, device] : devices) {
        device.sendCommand(command);
    }
}