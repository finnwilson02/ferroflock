/**
 * test_keyboard_control.cpp
 * 
 * Purpose: Test program for KeyboardControl functionality with a Tello drone
 * 
 * This is a standalone test that provides a clean interface for controlling
 * a Tello drone with keyboard input, without debug logging interference.
 */

#include "../include/keyboard_control.h"
#include "../include/tello_controller.h"
#include "../include/logger.h"  // Need to include for type definitions
#include <iostream>
#include <signal.h>
#include <thread>
#include <atomic>

// Define required global variables for linking
bool g_debug_enabled = false;  // Disable debug output
Logger* g_logger = nullptr;    // Null logger to silence outputs

// Global variables for coordination
TelloController* g_controller = nullptr;
KeyboardControl* g_keyboard_control = nullptr;
std::atomic<bool> g_running{true};

// Signal handler for clean shutdown
void signalHandler(int signal) {
    std::cout << "\nReceived signal " << signal << ", stopping keyboard control..." << std::endl;
    
    g_running = false;
    
    // Stop the keyboard control if it exists
    if (g_keyboard_control) {
        g_keyboard_control->stop();
    }
    
    // Clean up the controller if it exists
    if (g_controller) {
        g_controller->cleanup();
    }
    
    exit(signal);
}

int main() {
    // Register signal handlers
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
    
    // Define drone IP
    std::string drone_ip = "192.168.1.103";
    
    std::cout << "Connecting to drone at " << drone_ip << "..." << std::endl;
    
    // Create and initialize the TelloController
    TelloController controller;
    g_controller = &controller;
    
    // Initialize with skip_reboot=true for faster testing
    if (!controller.initialize(drone_ip, true)) {
        std::cerr << "Failed to initialize drone at " << drone_ip << std::endl;
        return 1;
    }
    
    std::cout << "Drone initialized successfully." << std::endl;
    
    // Print control instructions once
    std::cout << "\nController running. Controls:\n"
              << "  't' - takeoff\n"
              << "  'l' - land\n"
              << "  'q' - quit\n"
              << "  'w/a/s/d' - forward/left/backward/right\n"
              << "  'up/down' - increase/decrease altitude\n"
              << "  'left/right' - yaw counter-clockwise/clockwise\n"
              << "  'space' - hover in place (stop all movement)\n" 
              << std::endl;
    
    // Create KeyboardControl and start it
    KeyboardControl keyboard_control(controller);
    g_keyboard_control = &keyboard_control;
    keyboard_control.start();
    
    // Simple way to wait for KeyboardControl to exit (when user presses 'q')
    // Sleep in small increments to allow for quick response to signals
    while (g_running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    std::cout << "Keyboard control stopped, cleaning up..." << std::endl;
    
    // Clean up - land and reboot the drone
    controller.cleanup();
    
    std::cout << "Test completed successfully." << std::endl;
    return 0;
}