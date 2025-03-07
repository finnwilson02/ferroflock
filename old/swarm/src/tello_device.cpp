#include "tello_device.hpp"
#include <iostream>

TelloDevice::TelloDevice(const std::string& ip, const std::string& mac, int port)
    : ip(ip), mac(mac), port(port), tello(std::make_unique<ctello::Tello>()) {
}

bool TelloDevice::initialize() {
    if (!tello->Bind(port)) {
        std::cerr << "Failed to bind to port " << port << " for Tello at " << ip << std::endl;
        return false;
    }
    
    // Send initial command to specific Tello
    // Note: You might need to implement specific addressing here depending on your network setup
    std::cout << "Connected to Tello at " << ip << " (" << mac << ")" << std::endl;
    return true;
}

bool TelloDevice::sendCommand(const std::string& command) {
    if (!busy) {
        current_command = command;
        tello->SendCommand(command);
        busy = true;
        return true;
    }
    return false;
}

std::optional<std::string> TelloDevice::receiveResponse() {
    auto response = tello->ReceiveResponse();
    if (response) {
        busy = false;
    }
    return response;
}