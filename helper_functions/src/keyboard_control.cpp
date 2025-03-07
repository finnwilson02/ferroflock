/**
 * keyboard_control.cpp
 * 
 * Purpose: Implementation of keyboard control functionality for Tello drones
 * 
 * Data Flow:
 *   Input: Keyboard input from terminal
 *   Output: Control commands to Tello drones via TelloController
 * 
 * This implementation handles keyboard input processing, terminal configuration,
 * and sends drone control commands based on key presses.
 */

#include "../include/keyboard_control.h"
#include "../include/logger.h"
#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <chrono>
#include <algorithm> // For std::min, std::max

// External reference to global logger
extern Logger* g_logger;

// Constructor for controlling all drones
KeyboardControl::KeyboardControl(TelloController& controller)
    : controller_(controller), running_(false), drone_ip_("") {
    LOG_INFO("KeyboardControl initialized for all drones");
}

// Constructor for controlling a specific drone by IP
KeyboardControl::KeyboardControl(TelloController& controller, const std::string& drone_ip)
    : controller_(controller), running_(false), drone_ip_(drone_ip) {
    LOG_INFO("KeyboardControl initialized for drone " + drone_ip);
}

// Destructor
KeyboardControl::~KeyboardControl() {
    stop();
    LOG_INFO("KeyboardControl destroyed");
}

// Start keyboard control in a separate thread
void KeyboardControl::start() {
    if (running_) return;
    running_ = true;
    keyboard_thread_ = std::thread(&KeyboardControl::keyboardLoop, this);
    LOG_INFO("KeyboardControl started");
}

// Stop keyboard control thread
void KeyboardControl::stop() {
    running_ = false;
    if (keyboard_thread_.joinable()) {
        keyboard_thread_.join();
    }
    LOG_INFO("KeyboardControl stopped");
}

// Reset all speed levels to zero
void KeyboardControl::resetSpeedLevels() {
    fb_level_ = 0;
    lr_level_ = 0;
    ud_level_ = 0;
    yaw_level_ = 0;
}

// Main keyboard input processing loop
void KeyboardControl::keyboardLoop() {
    // Save terminal settings
    tcgetattr(STDIN_FILENO, &orig_termios_);
    
    // Configure terminal for non-blocking input
    struct termios raw = orig_termios_;
    raw.c_lflag &= ~(ICANON | ECHO);
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 0;
    tcsetattr(STDIN_FILENO, TCSAFLUSH, &raw);
    fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);

    // Print control instructions
    std::cout << "==== Keyboard Control Started ====" << std::endl;
    std::cout << "Controls:" << std::endl;
    std::cout << "  't' - takeoff" << std::endl;
    std::cout << "  'l' - land" << std::endl;
    std::cout << "  'q' - quit keyboard control" << std::endl;
    std::cout << "  'w/s' - increase/decrease forward speed" << std::endl;
    std::cout << "  'a/d' - increase left/right speed" << std::endl;
    std::cout << "  'Arrow Up/Down' - increase up/down speed" << std::endl;
    std::cout << "  'Arrow Left/Right' - increase rotation left/right" << std::endl;
    std::cout << "  'space' - hover (stop all movement)" << std::endl;
    std::cout << "  'enter' - emergency stop" << std::endl;
    std::cout << "Each press adjusts speed by 1 level (max ±10, maps to ±50 units)" << std::endl;
    std::cout << "Current levels are maintained until changed or reset" << std::endl;
    std::cout << "===============================" << std::endl;

    // Reset speed levels at startup
    resetSpeedLevels();

    // Control loop
    while (running_) {
        char key = getKey();
        if (key != -1) {
            if (key == 'q') {
                // Exit keyboard control
                running_ = false;
                break;
            } else if (key == 't') {
                // Takeoff
                if (drone_ip_.empty()) {
                    controller_.sendCommandToAll("takeoff");
                } else {
                    controller_.sendCommand(drone_ip_, "takeoff");
                }
                if (g_logger && g_logger->isOpen()) {
                    g_logger->logCommand("takeoff", 1.0, std::chrono::system_clock::to_time_t(std::chrono::system_clock::now()));
                }
            } else if (key == 'l') {
                // Land
                if (drone_ip_.empty()) {
                    controller_.sendCommandToAll("land");
                } else {
                    controller_.sendCommand(drone_ip_, "land");
                }
                if (g_logger && g_logger->isOpen()) {
                    g_logger->logCommand("land", 1.0, std::chrono::system_clock::to_time_t(std::chrono::system_clock::now()));
                }
            } else if (key == 'w') {
                // Increase forward speed
                fb_level_ = std::min(fb_level_ + 1, 10);
            } else if (key == 's') {
                // Increase backward speed
                fb_level_ = std::max(fb_level_ - 1, -10);
            } else if (key == 'a') {
                // Increase left speed
                lr_level_ = std::max(lr_level_ - 1, -10);
            } else if (key == 'd') {
                // Increase right speed
                lr_level_ = std::min(lr_level_ + 1, 10);
            } else if (key == KEY_UP) {
                // Increase up speed
                ud_level_ = std::min(ud_level_ + 1, 10);
            } else if (key == KEY_DOWN) {
                // Increase down speed
                ud_level_ = std::max(ud_level_ - 1, -10);
            } else if (key == KEY_LEFT) {
                // Increase rotate left speed
                yaw_level_ = std::max(yaw_level_ - 1, -10);
            } else if (key == KEY_RIGHT) {
                // Increase rotate right speed
                yaw_level_ = std::min(yaw_level_ + 1, 10);
            } else if (key == ' ') {
                // Hover (stop all movement)
                resetSpeedLevels();
            } else if (key == '\n' || key == '\r') {
                // Emergency stop
                if (drone_ip_.empty()) {
                    controller_.sendCommandToAll("emergency");
                } else {
                    controller_.sendCommand(drone_ip_, "emergency");
                }
                resetSpeedLevels();
                if (g_logger && g_logger->isOpen()) {
                    g_logger->logCommand("emergency", 1.0, std::chrono::system_clock::to_time_t(std::chrono::system_clock::now()));
                }
            }
            
            // Print current speed levels
            std::cout << "\rSpeed levels - FB: " << fb_level_ << " LR: " << lr_level_ 
                      << " UD: " << ud_level_ << " YAW: " << yaw_level_ << "     " << std::flush;
        }

        // Always send RC command based on current speed levels
        sendRCCommand(
            levelToSpeed(lr_level_),
            levelToSpeed(fb_level_),
            levelToSpeed(ud_level_),
            levelToSpeed(yaw_level_)
        );

        // Sleep to maintain 50Hz update rate (20ms)
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    // Restore terminal settings
    tcsetattr(STDIN_FILENO, TCSAFLUSH, &orig_termios_);
    std::cout << "\nKeyboard control stopped." << std::endl;
}

// Get a key press, handling special keys like arrow keys
char KeyboardControl::getKey() {
    char c;
    int result = read(STDIN_FILENO, &c, 1);
    if (result > 0) {
        if (c == 27) { // ESC sequence (for arrow keys)
            char next;
            result = read(STDIN_FILENO, &next, 1);
            if (result > 0 && next == '[') {
                result = read(STDIN_FILENO, &next, 1);
                if (result > 0) {
                    switch (next) {
                        case 'A': return KEY_UP;    // Up arrow
                        case 'B': return KEY_DOWN;  // Down arrow
                        case 'C': return KEY_RIGHT; // Right arrow
                        case 'D': return KEY_LEFT;  // Left arrow
                    }
                }
            }
            return 27; // Just ESC key
        }
        return c;
    }
    return -1; // No key pressed
}

// Send RC command to specific drone or all drones
void KeyboardControl::sendRCCommand(int lr, int fb, int ud, int yaw) {
    std::string command = "rc " + std::to_string(lr) + " " + 
                                 std::to_string(fb) + " " + 
                                 std::to_string(ud) + " " + 
                                 std::to_string(yaw);
    
    // If drone_ip_ is empty, send to all drones, otherwise send to specific drone
    if (drone_ip_.empty()) {
        controller_.sendCommandToAll(command);
    } else {
        controller_.sendCommand(drone_ip_, command);
    }
    
    // Don't log RC commands to avoid filling the log with frequent updates
}