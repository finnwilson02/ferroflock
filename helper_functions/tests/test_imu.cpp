// tests/test_imu.cpp
// Test script to retrieve and log IMU data from Tello drones

#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <vector>
#include <signal.h>
#include <cstdlib>
#include "../include/tello_controller.h"
#include "../include/tello_imu_handler.h"
#include "../include/logger.h"
#include "../include/menu.h"

// Define the global debug flag
bool g_debug_enabled = true;

// Global flag to control program execution, volatile for signal handling
volatile bool g_running = true;

// Global pointers for cleanup in signal handler
TelloController* g_controller = nullptr;
Logger* g_logger = nullptr;
TelloIMUHandler* g_imu_handler = nullptr;

// Cleanup function for proper resource management
void cleanup() {
    std::cout << "Cleaning up resources...\n";
    
    if (g_imu_handler) {
        delete g_imu_handler;
        g_imu_handler = nullptr;
    }
    
    if (g_controller) {
        g_controller->cleanup();
        delete g_controller;
        g_controller = nullptr;
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

int main() {
    // Register signal handler and cleanup function
    signal(SIGINT, signalHandler);
    std::atexit(cleanup);
    
    // Initialize controller
    g_controller = new TelloController();
    
    // Initialize OptiTrack and Menu
    OptiTrack optitrack;
    Menu menu(optitrack, *g_controller);
    menu.initialize();
    
    // Create data directory and logger
    Logger::createDataDirectory("data");
    std::string log_filename = "data/" + Logger::createUniqueFilename("imu_test", ".csv");
    g_logger = new Logger(log_filename);
    
    if (!g_logger->isOpen()) {
        std::cerr << "Failed to open log file: " << log_filename << "\n";
        return 1;
    }
    
    std::cout << "Logger initialized with file: " << log_filename << "\n";
    
    // Load drones from JSON file
    std::vector<DroneData> drones = menu.loadDronesFromJSON("../drone_info/dji_devices.json");
    
    if (drones.empty()) {
        std::cerr << "No drones found in configuration file.\n";
        return 1;
    }
    
    // List available drones
    std::cout << "Available drones:\n";
    for (size_t i = 0; i < drones.size(); ++i) {
        std::cout << i + 1 << ". " << drones[i].name << " (" << drones[i].ip << ")\n";
    }
    
    // Prompt user to select a drone
    size_t choice = 0;
    std::cout << "Select a drone (1-" << drones.size() << "): ";
    std::cin >> choice;
    
    if (choice < 1 || choice > drones.size()) {
        std::cerr << "Invalid choice. Please enter a number between 1 and " << drones.size() << ".\n";
        return 1;
    }
    
    // Get selected drone IP
    std::string selected_ip = drones[choice - 1].ip;
    std::string selected_name = drones[choice - 1].name;
    std::cout << "Selected drone: " << selected_name << " at " << selected_ip << "\n";
    
    // Create IMU handler for the selected drone
    g_imu_handler = new TelloIMUHandler(*g_controller, selected_ip);
    
    // Initialize the IMU handler to connect to the drone
    if (!g_imu_handler->initialize()) {
        std::cerr << "Failed to initialize IMU handler for drone at " << selected_ip << "\n";
        return 1;
    }
    
    std::cout << "Successfully initialized IMU handler for drone at " << selected_ip << "\n";
    
    // Logging duration
    const int logging_duration_seconds = 10;
    
    // Start logging loop
    auto start_time = std::chrono::steady_clock::now();
    int iteration = 0;
    
    std::cout << "Starting IMU data logging for " << logging_duration_seconds 
              << " seconds (press Ctrl+C to stop)...\n";
    
    while (g_running && 
           std::chrono::duration_cast<std::chrono::seconds>(
               std::chrono::steady_clock::now() - start_time).count() < logging_duration_seconds) {
        
        // Check if IMU data is valid
        if (!g_imu_handler->isDataValid()) {
            std::cerr << "[WARNING] IMU data is stale or not received for "
                      << selected_ip << "\n";
        }
        
        // Create data point for logging
        DataPoint data(selected_ip);
        data.timestamp = std::chrono::system_clock::now();
        data.imu_yaw = g_imu_handler->getYaw();
        data.imu_pitch = g_imu_handler->getPitch();
        data.imu_roll = g_imu_handler->getRoll();
        data.imu_agx = g_imu_handler->getAgx();
        data.imu_agy = g_imu_handler->getAgy();
        data.imu_agz = g_imu_handler->getAgz();
        
        // Log data point
        g_logger->logData(data);
        
        // Print IMU data to console every 10 iterations (1 second)
        if (iteration % 10 == 0) {
            std::cout << std::fixed << std::setprecision(2);
            std::cout << "IMU Data: "
                      << "Yaw=" << data.imu_yaw 
                      << ", Pitch=" << data.imu_pitch 
                      << ", Roll=" << data.imu_roll
                      << ", Agx=" << data.imu_agx 
                      << ", Agy=" << data.imu_agy 
                      << ", Agz=" << data.imu_agz << "\n";
        }
        
        iteration++;
        
        // Sleep for 100ms (10Hz update rate)
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    // Ensure all data is written
    g_logger->flush();
    
    std::cout << "Logging complete. Data saved to " << log_filename << "\n";
    std::cout << "Logged " << iteration << " data points.\n";
    
    // Cleanup is handled by atexit and signal handler
    return 0;
}