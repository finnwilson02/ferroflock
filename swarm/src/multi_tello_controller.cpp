#include "ctello.h"
#include <opencv2/opencv.hpp>
#include <vector>
#include <map>
#include <string>
#include <thread>
#include <mutex>
#include <atomic>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

// Store DJI MAC addresses for filtering
const std::vector<std::string> DJI_MAC_PREFIXES = {"34:D2:62"};

struct TelloDevice {
    std::string ip;
    std::string mac;
    std::unique_ptr<ctello::Tello> tello;
    bool busy{false};
    std::string current_command;
};

class MultiTelloController {
private:
    std::vector<TelloDevice> tellos;
    std::atomic<bool> running{true};
    std::mutex command_mutex;
    
    // Keyboard handling from your original code
    struct KeyboardInput {
        struct termios orig_termios;
        
        KeyboardInput() {
            tcgetattr(STDIN_FILENO, &orig_termios);
            struct termios raw = orig_termios;
            raw.c_lflag &= ~(ICANON | ECHO);
            raw.c_cc[VMIN] = 0;
            raw.c_cc[VTIME] = 0;
            tcsetattr(STDIN_FILENO, TCSAFLUSH, &raw);
            fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);
        }
        
        ~KeyboardInput() {
            tcsetattr(STDIN_FILENO, TCSAFLUSH, &orig_termios);
        }
        
        char getKey() {
            char c;
            int result = read(STDIN_FILENO, &c, 1);
            return result > 0 ? c : -1;
        }
    };

    bool isDJIDevice(const std::string& mac) {
        for (const auto& prefix : DJI_MAC_PREFIXES) {
            if (mac.substr(0, prefix.length()) == prefix) {
                return true;
            }
        }
        return false;
    }

    std::vector<std::pair<std::string, std::string>> discoverTellos() {
        std::vector<std::pair<std::string, std::string>> devices;
        // Here you would implement the actual network discovery
        // For your specific network, we can hardcode the discovered IPs
        devices.push_back({"192.168.1.104", "34:D2:62:ED:FF:BC"});
        devices.push_back({"192.168.1.106", "34:D2:62:ED:F1:61"});
        devices.push_back({"192.168.1.107", "34:D2:62:EE:00:50"});
        devices.push_back({"192.168.1.108", "34:D2:62:EE:01:B4"});
        devices.push_back({"192.168.1.109", "34:D2:62:EE:00:06"});
        devices.push_back({"192.168.1.110", "34:D2:62:ED:F3:CE"});
        return devices;
    }

public:
    bool initialize() {
        auto discovered_devices = discoverTellos();
        
        for (const auto& device : discovered_devices) {
            if (isDJIDevice(device.second)) {
                TelloDevice tello_device;
                tello_device.ip = device.first;
                tello_device.mac = device.second;
                tello_device.tello = std::make_unique<ctello::Tello>();
                
                // Configure for station mode
                if (!tello_device.tello->Bind(device.first)) {
                    std::cerr << "Failed to bind to Tello at " << device.first << std::endl;
                    continue;
                }
                
                tellos.push_back(std::move(tello_device));
                std::cout << "Connected to Tello at " << device.first << " (" << device.second << ")" << std::endl;
            }
        }
        
        return !tellos.empty();
    }

    void sendCommandToAll(const std::string& command) {
        std::lock_guard<std::mutex> lock(command_mutex);
        for (auto& tello : tellos) {
            if (!tello.busy) {
                tello.current_command = command;
                tello.tello->SendCommand(command);
                tello.busy = true;
            }
        }
    }

    void run() {
        KeyboardInput keyboard;
        const int speed = 50;
        
        std::cout << "\nControls for all drones:\n"
                  << "T: Takeoff\n"
                  << "L: Land\n"
                  << "I/K: Forward/Back\n"
                  << "J/H: Left/Right\n"
                  << "W/S: Up/Down\n"
                  << "A/D: Rotate Left/Right\n"
                  << "Space: Hover/Stop\n"
                  << "Q: Quit\n\n";

        // Start response handling thread
        std::thread response_thread([this]() {
            while (running) {
                for (auto& tello : tellos) {
                    if (auto response = tello.tello->ReceiveResponse()) {
                        std::cout << "Tello " << tello.ip << ": " << *response << std::endl;
                        tello.busy = false;
                    }
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        });

        // Main control loop
        while (running) {
            char key = keyboard.getKey();
            if (key != -1) {
                std::string command;
                
                switch (key) {
                    case 't':
                        command = "takeoff";
                        break;
                    case 'l':
                        command = "land";
                        break;
                    case 'w':
                        command = "rc 0 0 " + std::to_string(speed) + " 0";
                        break;
                    case 's':
                        command = "rc 0 0 " + std::to_string(-speed) + " 0";
                        break;
                    case 'a':
                        command = "rc 0 0 0 " + std::to_string(-speed);
                        break;
                    case 'd':
                        command = "rc 0 0 0 " + std::to_string(speed);
                        break;
                    case 'i':
                        command = "rc 0 " + std::to_string(speed) + " 0 0";
                        break;
                    case 'k':
                        command = "rc 0 " + std::to_string(-speed) + " 0 0";
                        break;
                    case 'j':
                        command = "rc " + std::to_string(-speed) + " 0 0 0";
                        break;
                    case 'h':
                        command = "rc " + std::to_string(speed) + " 0 0 0";
                        break;
                    case ' ':
                        command = "rc 0 0 0 0";
                        break;
                    case 'q':
                        sendCommandToAll("land");
                        running = false;
                        break;
                }
                
                if (!command.empty()) {
                    sendCommandToAll(command);
                }
            }
            
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        
        response_thread.join();
    }
};

int main() {
    MultiTelloController controller;
    
    if (!controller.initialize()) {
        std::cerr << "Failed to initialize any Tello connections!" << std::endl;
        return 1;
    }
    
    controller.run();
    return 0;
}