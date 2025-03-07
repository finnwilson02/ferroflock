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

    void keyboardLoop();
    char getKey();
    void sendRCCommand(int lr, int fb, int ud, int yaw);

    // Constants for special keys
    static constexpr char KEY_UP = 128;
    static constexpr char KEY_DOWN = 129;
    static constexpr char KEY_LEFT = 130;
    static constexpr char KEY_RIGHT = 131;
};

#endif // KEYBOARD_CONTROL_H