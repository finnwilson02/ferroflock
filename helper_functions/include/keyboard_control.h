/**
 * keyboard_control.h
 * 
 * Purpose: Provides keyboard control functionality for Tello drones
 * 
 * Data Flow:
 *   Input: Keyboard input from terminal
 *   Output: Control commands to Tello drones via TelloController
 * 
 * This module handles keyboard input, maps keys to drone commands,
 * and sends appropriate commands to the TelloController for execution.
 */

#ifndef KEYBOARD_CONTROL_H
#define KEYBOARD_CONTROL_H

#include "tello_controller.h"
#include <thread>
#include <atomic>
#include <termios.h>

class KeyboardControl {
public:
    KeyboardControl(TelloController& controller);
    ~KeyboardControl();

    void start();
    void stop();

private:
    TelloController& controller_;
    std::thread keyboard_thread_;
    std::atomic<bool> running_;
    struct termios orig_termios_;
    
    // Speed level tracking (-10 to 10 for each axis)
    int fb_level_{0};  // Forward-Backward
    int lr_level_{0};  // Left-Right
    int ud_level_{0};  // Up-Down
    int yaw_level_{0}; // Yaw rotation

    void keyboardLoop();
    char getKey();
    void sendRCCommand(int lr, int fb, int ud, int yaw);
    
    // Convert level to actual speed value (level * 5)
    int levelToSpeed(int level) const { return level * 5; }
    
    // Reset all speed levels to zero
    void resetSpeedLevels();

    // Constants for special keys
    static constexpr char KEY_UP = 128;
    static constexpr char KEY_DOWN = 129;
    static constexpr char KEY_LEFT = 130;
    static constexpr char KEY_RIGHT = 131;
};

#endif // KEYBOARD_CONTROL_H