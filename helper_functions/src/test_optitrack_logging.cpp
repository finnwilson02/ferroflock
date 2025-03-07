// test_optitrack_logging.cpp
// Standalone test script to log OptiTrack data for one active tracker

#include <iostream>          // For console output
#include <signal.h>          // For signal handling (Ctrl+C)
#include <chrono>            // For timing and sleep
#include <thread>            // For sleep functionality
#include <memory>            // For smart pointers if needed later
#include <cstdlib>           // For atexit
#include "optitrack.h"       // OptiTrack class for tracker data
#include "logger.h"          // Logger class for saving data
#include "test_optitrack_logging.h" // Header for this file

// Define the global debug flag
bool g_debug_enabled = false;

// Global flag to control program execution, volatile for signal handling
volatile bool g_running = true;

// Global pointers for cleanup in signal handler
OptiTrack* g_optitrack = nullptr;
Logger* g_logger = nullptr;

// Cleanup function for proper resource management
void cleanup() {
    std::cout << "Cleaning up resources...\n";
    if (g_optitrack) {
        g_optitrack->stopVisualization();
        delete g_optitrack;
        g_optitrack = nullptr;
    }
    if (g_logger) {
        g_logger->close();
        delete g_logger;
        g_logger = nullptr;
    }
    std::cout << "Cleanup complete.\n";
}

// Signal handler for graceful exit on Ctrl+C
void signalHandler(int signal) {
    if (signal == SIGINT) {
        std::cout << "\nReceived Ctrl+C, shutting down...\n";
        g_running = false;
        cleanup(); // Immediate cleanup on signal
        std::exit(0); // Exit immediately
    }
}

// Select the first tracker with non-zero position
std::string selectActiveTracker(OptiTrack& optitrack) {
    std::this_thread::sleep_for(std::chrono::seconds(2)); // Wait for trackers to appear
    auto trackers = optitrack.getActiveTrackers();
    for (const auto& tracker : trackers) {
        double x = optitrack.getXPosition(tracker);
        double y = optitrack.getYPosition(tracker);
        double z = optitrack.getZPosition(tracker);
        if (x != 0.0 || y != 0.0 || z != 0.0) {
            return tracker;
        }
    }
    return ""; // Empty string if no active tracker found
}

int main() {
    // Register signal handler and cleanup function
    signal(SIGINT, signalHandler);
    std::atexit(cleanup);

    // Initialize OptiTrack
    g_optitrack = new OptiTrack();
    g_optitrack->initialize();
    g_optitrack->startVisualization(); // Start visualization (already working)
    std::cout << "OptiTrack initialized and visualization started.\n";

    // Create data directory and logger
    Logger::createDataDirectory("data"); // Ensure data directory exists
    std::string log_filename = "data/test_optitrack_" + 
                              Logger::createUniqueFilename("log", ".csv");
    g_logger = new Logger(log_filename);
    if (!g_logger->isOpen()) {
        std::cerr << "Failed to open log file: " << log_filename << "\n";
        return 1; // Cleanup handled by atexit
    }
    std::cout << "Logger initialized with file: " << log_filename << "\n";

    // Select active tracker
    std::string active_tracker = selectActiveTracker(*g_optitrack);
    if (active_tracker.empty()) {
        std::cerr << "No active tracker with non-zero position found.\n";
        return 1; // Cleanup handled by atexit
    }
    std::cout << "Selected active tracker: " << active_tracker << "\n";

    // Logging loop (30 seconds or until Ctrl+C)
    auto start_time = std::chrono::steady_clock::now();
    int iteration = 0;
    std::cout << "Starting logging for 30 seconds (press Ctrl+C to stop)...\n";
    while (g_running && 
           std::chrono::duration_cast<std::chrono::seconds>(
               std::chrono::steady_clock::now() - start_time).count() < 30) {
        DataPoint data(active_tracker);
        data.x = g_optitrack->getXPosition(active_tracker);
        data.y = g_optitrack->getYPosition(active_tracker);
        data.z = g_optitrack->getZPosition(active_tracker);
        data.yaw_raw = g_optitrack->getRawYaw(active_tracker);
        data.yaw_corrected = g_optitrack->getYaw(active_tracker);
        g_logger->logData(data);

        if (iteration % 10 == 0) {
            std::cout << "Logged: x=" << data.x << ", y=" << data.y << ", z=" << data.z 
                      << ", yaw=" << data.yaw_corrected << "\n";
        }
        iteration++;

        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 10 Hz
    }
    g_logger->flush(); // Ensure all data is written
    std::cout << "Logging complete. Data saved to " << log_filename << "\n";

    // Normal cleanup handled by atexit or signal handler
    return 0;
}