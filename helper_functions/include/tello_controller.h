/**
 * tello_controller.h
 * 
 * Purpose: Manages communication with DJI Tello drones
 * 
 * Data Flow:
 *   Input: Command strings from application logic
 *   Output: UDP commands sent to drones, responses received from drones
 * 
 * This module handles initialization, command transmission, response handling,
 * and cleanup for multiple Tello drones across the network.
 */

#ifndef TELLO_CONTROLLER_H
#define TELLO_CONTROLLER_H

#include <string>
#include <vector>
#include <map>
#include <thread>
#include <mutex>
#include <chrono>
#include <functional>
#include <optional>
#include <iostream>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <unistd.h>
#include <fcntl.h>

class TelloController {
public:
    // Constructor/Destructor
    TelloController();
    ~TelloController();

    // Initialize a drone with its IP address
    bool initialize(const std::string& ip, bool skip_reboot = false);
    
    // Send a command to a drone
    bool sendCommand(const std::string& ip, const std::string& command, bool skip_reboot = false);
    
    // Receive a response from a drone (with optional timeout)
    std::optional<std::string> receiveResponse(const std::string& ip, int timeout_ms = 100);
    
    // Reboot a specific drone
    bool rebootDrone(const std::string& ip, int port = 8889);
    
    // Check if a drone is responding to ping
    bool pingDrone(const std::string& ip);
    
    // Perform cleanup (land all drones, close sockets, etc.)
    void cleanup();
    
    // Send a command to all drones simultaneously
    void sendCommandToAll(const std::string& command);

private:
    // Private inner class to handle individual Tello devices
    class TelloDevice {
    public:
        std::string ip;
        int command_port{8889};  // Default command port
        int local_port;          // Unique local port for each drone
        int command_socket{-1};
        struct sockaddr_in command_addr{};
        bool socket_valid{false};
        bool needs_reboot{true}; // Always reboot at first
        
        // Constructor with parameters
        TelloDevice(std::string ip_addr = "", int cmd_port = 8889, int loc_port = 8890)
            : ip(ip_addr), command_port(cmd_port), local_port(loc_port),
              command_socket(-1), socket_valid(false), needs_reboot(false) {}
              
        // Move constructor
        TelloDevice(TelloDevice&& other) noexcept
            : ip(std::move(other.ip)),
              command_port(other.command_port),
              local_port(other.local_port),
              command_socket(other.command_socket),
              command_addr(other.command_addr),
              socket_valid(other.socket_valid),
              needs_reboot(other.needs_reboot) {
            std::cout << "[SOCKET] Moved socket " << command_socket << " from " << other.ip << std::endl;
            other.command_socket = -1;
            other.socket_valid = false;
        }

        // Move assignment operator
        TelloDevice& operator=(TelloDevice&& other) noexcept {
            if (this != &other) {
                if (command_socket >= 0) {
                    std::cout << "[SOCKET] Closing socket " << command_socket << " for " << ip << std::endl;
                    close(command_socket);
                }
                ip = std::move(other.ip);
                command_port = other.command_port;
                local_port = other.local_port;
                command_socket = other.command_socket;
                command_addr = other.command_addr;
                socket_valid = other.socket_valid;
                needs_reboot = other.needs_reboot;
                std::cout << "[SOCKET] Moved socket " << command_socket << " from " << other.ip << std::endl;
                other.command_socket = -1;
                other.socket_valid = false;
            }
            return *this;
        }
        
        // Destructor to close sockets
        ~TelloDevice();
        
        // Prevent copying
        TelloDevice(const TelloDevice&) = delete;
        TelloDevice& operator=(const TelloDevice&) = delete;
        
        // Initialize socket connections
        bool initializeSockets();
        
        // Send a command to the drone
        bool sendCommand(const std::string& cmd);
        
        // Re-initialize sockets and send a command
        bool reinitializeAndSend(const std::string& cmd);
        
        // Receive a response from the drone
        std::optional<std::string> receiveResponse();
        
        // Clean a command string to remove invalid characters
        std::string cleanCommand(const std::string& cmd);
    };
    
    // Map of IP addresses to TelloDevice objects
    std::map<std::string, TelloDevice> devices;
    
    // Counter for assigning unique local ports
    int next_port = 9000;
};

#endif // TELLO_CONTROLLER_H