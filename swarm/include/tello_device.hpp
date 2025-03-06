#pragma once

#include "ctello.h"
#include <memory>
#include <string>
#include <atomic>

class TelloDevice {
public:
    static constexpr int DEFAULT_PORT = 8889;  // Default Tello command port
    
    TelloDevice(const std::string& ip, const std::string& mac, int port = DEFAULT_PORT);
    
    bool initialize();
    bool sendCommand(const std::string& command);
    std::optional<std::string> receiveResponse();
    
    // Getters
    const std::string& getIP() const { return ip; }
    const std::string& getMAC() const { return mac; }
    bool isBusy() const { return busy; }
    const std::string& getCurrentCommand() const { return current_command; }

private:
    std::string ip;
    std::string mac;
    int port;
    std::unique_ptr<ctello::Tello> tello;
    std::atomic<bool> busy{false};
    std::string current_command;
};