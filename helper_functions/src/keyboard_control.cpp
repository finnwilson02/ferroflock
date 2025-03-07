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

// External reference to global logger
extern Logger* g_logger;

// Constructor
KeyboardControl::KeyboardControl(TelloController& controller)
    : controller_(controller), running_(false) {
    LOG_INFO("KeyboardControl initialized");
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
    std::cout << "  'w/s' - forward/backward" << std::endl;
    std::cout << "  'a/d' - left/right" << std::endl;
    std::cout << "  'Arrow Up/Down' - up/down" << std::endl;
    std::cout << "  'Arrow Left/Right' - rotate left/right" << std::endl;
    std::cout << "  'space' - hover (stop all movement)" << std::endl;
    std::cout << "===============================" << std::endl;

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
                controller_.sendCommandToAll("takeoff");
                if (g_logger && g_logger->isOpen()) {
                    g_logger->logCommand("takeoff", 1.0, std::chrono::system_clock::to_time_t(std::chrono::system_clock::now()));
                }
            } else if (key == 'l') {
                // Land
                controller_.sendCommandToAll("land");
                if (g_logger && g_logger->isOpen()) {
                    g_logger->logCommand("land", 1.0, std::chrono::system_clock::to_time_t(std::chrono::system_clock::now()));
                }
            } else if (key == 'w') {
                // Forward
                sendRCCommand(0, 50, 0, 0);
            } else if (key == 's') {
                // Backward
                sendRCCommand(0, -50, 0, 0);
            } else if (key == 'a') {
                // Left
                sendRCCommand(-50, 0, 0, 0);
            } else if (key == 'd') {
                // Right
                sendRCCommand(50, 0, 0, 0);
            } else if (key == KEY_UP) {
                // Up
                sendRCCommand(0, 0, 50, 0);
            } else if (key == KEY_DOWN) {
                // Down
                sendRCCommand(0, 0, -50, 0);
            } else if (key == KEY_LEFT) {
                // Rotate left
                sendRCCommand(0, 0, 0, -50);
            } else if (key == KEY_RIGHT) {
                // Rotate right
                sendRCCommand(0, 0, 0, 50);
            } else if (key == ' ') {
                // Hover (stop all movement)
                sendRCCommand(0, 0, 0, 0);
            }
        } else {
            // No key press - send hover command to maintain stability
            sendRCCommand(0, 0, 0, 0);
        }

        // Sleep to avoid excessive CPU usage and to limit RC command rate
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    // Restore terminal settings
    tcsetattr(STDIN_FILENO, TCSAFLUSH, &orig_termios_);
    std::cout << "Keyboard control stopped." << std::endl;
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

// Send RC command to all drones
void KeyboardControl::sendRCCommand(int lr, int fb, int ud, int yaw) {
    std::string command = "rc " + std::to_string(lr) + " " + 
                                 std::to_string(fb) + " " + 
                                 std::to_string(ud) + " " + 
                                 std::to_string(yaw);
    controller_.sendCommandToAll(command);
    
    // Don't log RC commands to avoid filling the log with frequent updates
}