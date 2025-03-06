#include "drone_controller.h"
#include <iostream>
#include <signal.h>
#include <cstdlib>

int main() {
    // Set environment variable to use X11 instead of Wayland for Qt
    setenv("QT_QPA_PLATFORM", "xcb", 1);
    // Setup signal handler to ensure clean exit on Ctrl+C
    struct sigaction sa;
    sa.sa_handler = [](int sig) {
        std::cout << "\nExiting..." << std::endl;
        exit(sig);
    };
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;
    sigaction(SIGINT, &sa, NULL);  // Handle Ctrl+C
    sigaction(SIGTERM, &sa, NULL); // Handle termination signal
    
    try {
        DroneController controller;
        controller.run();
    } catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
        return 1;
    } catch (...) {
        std::cerr << "Unknown exception occurred" << std::endl;
        return 1;
    }
    
    return 0;
}