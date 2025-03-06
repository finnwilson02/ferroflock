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
        
        // Destructor to close sockets
        ~TelloDevice();
        
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