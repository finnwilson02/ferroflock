#include "ctello.h"
#include <iostream>
#include <chrono>
#include <thread>

int main() {
    const char* LAB_SSID = "UAV-LAB-2.4G";
    const char* LAB_PASSWORD = "idontknow";
    
    ctello::Tello tello;
    
    // First connect to the Tello's default WiFi
    if (!tello.Bind()) {
        std::cerr << "Failed to bind to Tello" << std::endl;
        return 1;
    }

    // Enter command mode
    tello.SendCommand("command");
    auto response = tello.ReceiveResponse();
    if (!response || *response != "ok") {
        std::cerr << "Failed to enter command mode" << std::endl;
        return 1;
    }

    // Check battery just to verify connection
    tello.SendCommand("battery?");
    response = tello.ReceiveResponse();
    if (response) {
        std::cout << "Battery level: " << *response << "%" << std::endl;
    }

    // Set to AP mode with lab WiFi credentials
    std::string ap_command = "ap " + std::string(LAB_SSID) + " " + std::string(LAB_PASSWORD);
    std::cout << "Sending command: " << ap_command << std::endl;
    
    tello.SendCommand(ap_command);
    response = tello.ReceiveResponse();
    if (response) {
        std::cout << "AP mode response: " << *response << std::endl;
    }

    std::cout << "Configuration complete. Drone will attempt to connect to lab WiFi." << std::endl;
    std::cout << "You'll need to scan for its new IP address." << std::endl;
    
    return 0;
}