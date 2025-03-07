#pragma once

#include "tello_device.hpp"
#include "keyboard_input.hpp"
#include <vector>
#include <mutex>
#include <atomic>
#include <thread>

class MultiTelloController {
public:
    MultiTelloController();
    ~MultiTelloController();

    bool initialize();
    void run();
    void stop();

private:
    std::vector<std::unique_ptr<TelloDevice>> tellos;
    std::atomic<bool> running{true};
    std::mutex command_mutex;
    std::unique_ptr<std::thread> response_thread;
    KeyboardInput keyboard;

    static const std::vector<std::string> DJI_MAC_PREFIXES;
    static constexpr int DEFAULT_SPEED = 50;

    bool isDJIDevice(const std::string& mac);
    std::vector<std::pair<std::string, std::string>> discoverTellos();
    void sendCommandToAll(const std::string& command);
    void handleResponses();
    void processKeyboardInput();
    void displayControls();
};