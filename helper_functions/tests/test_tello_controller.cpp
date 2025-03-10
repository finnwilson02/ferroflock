#include "../include/tello_controller.h"
#include "../include/logger.h"
#include <iostream>
#include <thread>
#include <chrono>

// External references to global variables
extern Logger* g_logger;
extern bool g_debug_enabled;
extern LogLevel g_log_level;

int main() {
    // Create a logger instance
    g_logger = new Logger("tello_control_test.csv");
    g_logger->open("tello_control_test.csv");
    
    // Set log level to debug to see detailed logs
    g_log_level = LOG_LEVEL_DEBUG;
    g_debug_enabled = true;
    
    std::cout << "Starting Tello Controller 100Hz test..." << std::endl;
    
    // Create a TelloController instance
    TelloController controller;
    
    // IP address of the drone to test (update as needed)
    std::string drone_ip = "192.168.1.106";  // Use a valid drone IP
    
    // Initialize the drone
    std::cout << "Initializing drone at " << drone_ip << std::endl;
    if (!controller.initialize(drone_ip)) {
        std::cerr << "Failed to initialize drone at " << drone_ip << std::endl;
        delete g_logger;
        return 1;
    }
    
    // Enter SDK mode
    std::cout << "Sending SDK mode command..." << std::endl;
    controller.sendCommand(drone_ip, "command");
    
    // Send takeoff command to start flying
    std::cout << "Sending takeoff command..." << std::endl;
    controller.sendCommand(drone_ip, "takeoff");
    
    // Wait for takeoff to complete
    std::cout << "Waiting for takeoff to complete (5 seconds)..." << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(5));
    
    // Send RC commands in sequence
    std::cout << "Testing RC commands with 100Hz control loop..." << std::endl;
    
    // Go up for 0.5 seconds
    std::cout << "Going up..." << std::endl;
    controller.sendCommand(drone_ip, "rc 0 0 50 0");
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    // Go forward for 1 second
    std::cout << "Going forward..." << std::endl;
    controller.sendCommand(drone_ip, "rc 0 50 0 0");
    std::this_thread::sleep_for(std::chrono::seconds(1));
    
    // Turn right for 1 second
    std::cout << "Turning right..." << std::endl;
    controller.sendCommand(drone_ip, "rc 0 0 0 50");
    std::this_thread::sleep_for(std::chrono::seconds(1));
    
    // Stop movement
    std::cout << "Stopping..." << std::endl;
    controller.sendCommand(drone_ip, "rc 0 0 0 0");
    std::this_thread::sleep_for(std::chrono::seconds(1));
    
    // Land the drone
    std::cout << "Landing..." << std::endl;
    controller.sendCommand(drone_ip, "land");
    
    // Wait for landing to complete
    std::cout << "Waiting for landing to complete (5 seconds)..." << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(5));
    
    // Close the logger
    g_logger->close();
    delete g_logger;
    
    std::cout << "Test completed. Check 'tello_control_test.csv' for logged commands." << std::endl;
    
    return 0;
}