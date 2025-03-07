/**
 * main.cpp
 * 
 * Purpose: Main entry point for the ferroflock drone control application
 * 
 * Data Flow:
 *   Input: User commands, OptiTrack data, drone responses
 *   Output: Control commands to drones, logging data to files
 * 
 * This module initializes all system components (OptiTrack, TelloController,
 * Logger, Menu), handles signal interrupts, and coordinates the overall 
 * application flow.
 */

#include <iostream>
#include <signal.h>
#include <filesystem>
#include <string.h>
#include "../include/tello_controller.h"
#include "../include/logger.h"
#include "../include/optitrack.h"
#include "../include/menu.h"

// Global debug flag
bool g_debug_enabled = false;

// Global instances
TelloController* g_tello_controller = nullptr;
OptiTrack* g_optitrack = nullptr;
Logger* g_logger = nullptr;
Menu* g_menu = nullptr;

// Signal handler for graceful exit
void signalHandler(int signal) {
    LOG_INFO("Received signal " + std::to_string(signal) + ", cleaning up...");
    
    // Clean up resources
    if (g_tello_controller) {
        g_tello_controller->cleanup();
    }
    
    if (g_optitrack) {
        g_optitrack->stopVisualization();
    }
    
    if (g_logger) {
        g_logger->close();
    }
    
    LOG_INFO("Cleanup complete, exiting");
    exit(signal);
}

// Main cleanup function to be called at program exit
void cleanup() {
    LOG_INFO("Program exit detected, cleaning up...");
    
    // Clean up resources
    if (g_tello_controller) {
        g_tello_controller->cleanup();
        delete g_tello_controller;
        g_tello_controller = nullptr;
    }
    
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
    
    if (g_menu) {
        delete g_menu;
        g_menu = nullptr;
    }
    
    LOG_INFO("Cleanup complete");
}

void printUsage(const char* programName) {
    std::cout << "Usage: " << programName << " [options]" << std::endl;
    std::cout << "Options:" << std::endl;
    std::cout << "  -d, --debug    Enable debug output" << std::endl;
    std::cout << "  -h, --help     Display this help message" << std::endl;
}

int main(int argc, char** argv) {
    // Parse command-line arguments
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-d") == 0 || strcmp(argv[i], "--debug") == 0) {
            g_debug_enabled = true;
            std::cout << "Debug mode enabled" << std::endl;
        } else if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
            printUsage(argv[0]);
            return 0;
        } else {
            std::cerr << "Unknown option: " << argv[i] << std::endl;
            printUsage(argv[0]);
            return 1;
        }
    }
    
    LOG_INFO("Starting helper_functions application");
    
    // Register signal handlers
    signal(SIGINT, signalHandler);   // Ctrl+C
    signal(SIGTERM, signalHandler);  // Termination signal
    
    // Register cleanup function to be called at program exit
    std::atexit(cleanup);
    
    // Create data directory if it doesn't exist
    Logger::createDataDirectory("data");
    
    // Create a logger for the application
    std::string log_filename = "data/application_" + 
                             Logger::createUniqueFilename("log", ".csv");
    g_logger = new Logger(log_filename);
    
    // Create OptiTrack instance
    g_optitrack = new OptiTrack();
    g_optitrack->initialize();
    g_optitrack->startVisualization();
    
    // Create TelloController instance
    g_tello_controller = new TelloController();
    
    // Create menu instance
    g_menu = new Menu(*g_optitrack, *g_tello_controller);
    g_menu->initialize();
    
    // Main menu loop
    LOG_INFO("Entering main menu loop");
    try {
        while (true) {
            g_menu->display();
        }
    } catch (const std::exception& e) {
        LOG_ERROR("Exception in main loop: " + std::string(e.what()));
        return 1;
    } catch (...) {
        LOG_ERROR("Unknown exception in main loop");
        return 2;
    }
    
    return 0;
}