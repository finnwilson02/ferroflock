#include "ctello.h"
#include <iostream>
#include <vector>
#include <string>
#include <thread>
#include <mutex>
#include <atomic>
#include <queue>
#include <chrono>
#include <memory>
#include <algorithm>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <cstring>
#include <csignal>

// Forward declaration
class MultiTelloController;

// Global pointer for signal handling
MultiTelloController* g_controller = nullptr;

// Signal handler
void signalHandler(int signal);

class MultiTelloController {
private:
    // Keyboard handling
    class KeyboardInput {
    private:
        struct termios orig_termios;
        
    public:
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

    // Command queue entry
    struct Command {
        std::string cmd;
        bool requires_sync;
        std::chrono::steady_clock::time_point timestamp;
    };
    
    struct TelloDevice {
        std::string ip;
        int command_port{8889};  // Default command port
        int state_port{8890};    // Default state port
        int video_port{11111};   // Default video port
        int local_port;          // Unique local port for each drone
        
        int command_socket{-1};
        struct sockaddr_in command_addr{};
        
        bool command_acked{false};
        std::string current_command;
        std::chrono::steady_clock::time_point last_command_time;
        std::string last_error;
        int error_count{0};
        const int MAX_ERRORS = 3;
        bool socket_valid{false};
        
        enum class CommandState {
            IDLE,
            WAITING_ACK,
            EXECUTING,
            COMPLETED,
            ERROR
        } state{CommandState::IDLE};

        // Default constructor
        TelloDevice() = default;

        // Move constructor
        TelloDevice(TelloDevice&& other) noexcept {
            ip = std::move(other.ip);
            command_port = other.command_port;
            state_port = other.state_port;
            video_port = other.video_port;
            local_port = other.local_port;
            command_socket = other.command_socket;
            command_addr = other.command_addr;
            socket_valid = other.socket_valid;
            command_acked = other.command_acked;
            current_command = std::move(other.current_command);
            last_command_time = other.last_command_time;
            last_error = std::move(other.last_error);
            error_count = other.error_count;
            state = other.state;
            
            other.command_socket = -1;
            other.socket_valid = false;
        }

        // Move assignment operator
        TelloDevice& operator=(TelloDevice&& other) noexcept {
            if (this != &other) {
                if (command_socket >= 0) {
                    close(command_socket);
                }
                ip = std::move(other.ip);
                command_port = other.command_port;
                state_port = other.state_port;
                video_port = other.video_port;
                local_port = other.local_port;
                command_socket = other.command_socket;
                command_addr = other.command_addr;
                socket_valid = other.socket_valid;
                command_acked = other.command_acked;
                current_command = std::move(other.current_command);
                last_command_time = other.last_command_time;
                last_error = std::move(other.last_error);
                error_count = other.error_count;
                state = other.state;
                
                other.command_socket = -1;
                other.socket_valid = false;
            }
            return *this;
        }

        ~TelloDevice() {
            if (command_socket >= 0) {
                close(command_socket);
                command_socket = -1;
            }
        }

        bool initializeSockets() {
            // Close existing socket if any
            if (command_socket >= 0) {
                close(command_socket);
                command_socket = -1;
                socket_valid = false;
            }

            command_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
            if (command_socket < 0) {
                last_error = strerror(errno);
                std::cerr << "Failed to create socket for " << ip << ": " << last_error << std::endl;
                return false;
            }

            // Duplicate the socket to ensure unique descriptor
            int new_socket = dup(command_socket);
            close(command_socket);
            command_socket = new_socket;

            std::cout << "[SOCKET] Created new socket " << command_socket 
                    << " for " << ip << ":" << local_port << std::endl;

            // Enable socket reuse
            int reuse = 1;
            if (setsockopt(command_socket, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse)) < 0) {
                last_error = strerror(errno);
                std::cerr << "Failed to set SO_REUSEADDR for " << ip << ": " << last_error << std::endl;
                close(command_socket);
                return false;
            }

            // Set up local address
            struct sockaddr_in local_addr{};
            local_addr.sin_family = AF_INET;
            local_addr.sin_addr.s_addr = INADDR_ANY;
            local_addr.sin_port = htons(local_port);

            // Bind to local port
            if (bind(command_socket, (struct sockaddr *)&local_addr, sizeof(local_addr)) < 0) {
                last_error = strerror(errno);
                std::cerr << "Failed to bind socket for " << ip 
                        << ": " << last_error << std::endl;
                close(command_socket);
                return false;
            }

            // Set up remote address
            command_addr = {};
            command_addr.sin_family = AF_INET;
            command_addr.sin_addr.s_addr = inet_addr(ip.c_str());
            command_addr.sin_port = htons(command_port);

            // Set socket timeout
            struct timeval tv;
            tv.tv_sec = 0;
            tv.tv_usec = 100000; // 100ms timeout
            setsockopt(command_socket, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

            socket_valid = true;
            return true;
        }

        bool sendCommand(const std::string& cmd) {
            if (!socket_valid || command_socket < 0) {
                std::cerr << "[ERROR] Invalid socket state for " << ip << std::endl;
                return reinitializeAndSend(cmd);
            }
            
            // Clean command just like in working code
            std::string clean_cmd = cleanCommand(cmd);
            
            std::cout << "[SEND] IP: " << ip << " Port: " << local_port 
                    << " Socket: " << command_socket 
                    << " Command: " << clean_cmd << std::endl;
            
            ssize_t sent = sendto(command_socket, clean_cmd.c_str(), clean_cmd.length(), 0,
                                (struct sockaddr *)&command_addr, sizeof(command_addr));
            
            if (sent < 0) {
                last_error = strerror(errno);
                error_count++;
                std::cerr << "Send failed for " << ip << ": " << last_error << std::endl;
                socket_valid = false;
                return reinitializeAndSend(clean_cmd);
            }
            
            std::cout << "[SUCCESS] Sent " << sent << " bytes to " << ip << std::endl;
            error_count = 0;
            return true;
        }

        bool reinitializeAndSend(const std::string& cmd) {
            if (initializeSockets()) {
                return sendCommand(cmd);
            }
            return false;
        }

        // Add timing control
        std::chrono::steady_clock::time_point last_rc_command;
        static constexpr int RC_COMMAND_INTERVAL_MS = 20;  // 50Hz max

        bool isValidResponse(const std::string& response, const struct sockaddr_in& from_addr) {
            // Verify sender
            if (from_addr.sin_addr.s_addr != command_addr.sin_addr.s_addr ||
                from_addr.sin_port != command_addr.sin_port) {
                return false;
            }

            // Filter out error messages when not expecting a response
            if (response == "error" && 
                (state != CommandState::WAITING_ACK && 
                state != CommandState::EXECUTING)) {
                return false;
            }

            return true;
        }

        bool shouldSendRCCommand() {
            auto now = std::chrono::steady_clock::now();
            if (current_command.find("rc") == 0 &&
                now - last_rc_command < std::chrono::milliseconds(RC_COMMAND_INTERVAL_MS)) {
                return false;
            }
            last_rc_command = now;
            return true;
        }

        std::optional<std::string> receiveResponse() {
            if (!socket_valid || command_socket < 0) return std::nullopt;
            
            char buffer[1024];
            struct sockaddr_in from_addr;
            socklen_t from_len = sizeof(from_addr);
            
            ssize_t received = recvfrom(command_socket, buffer, sizeof(buffer)-1, 0,
                                    (struct sockaddr *)&from_addr, &from_len);
            
            if (received < 0) {
                if (errno != EAGAIN && errno != EWOULDBLOCK) {
                    std::cerr << "Receive error for " << ip << ": " 
                            << strerror(errno) << std::endl;
                }
                return std::nullopt;
            }
            
            buffer[received] = '\0';
            std::string response(buffer);

            // Filter invalid responses
            if (!isValidResponse(response, from_addr)) {
                return std::nullopt;
            }
            
            std::cout << "[RESPONSE] IP: " << ip << " Response: " << response 
                    << " State: " << static_cast<int>(state) << std::endl;
            return response;
        }

        // Add this with the other helper methods
        std::string cleanCommand(const std::string& cmd) {
            std::string clean;
            for(char c : cmd) {
                if (isalnum(c) || c == ' ' || c == '?' || c == '-') {
                    clean += c;
                }
            }
            return clean;
        }
        
    };
     
    std::vector<TelloDevice> tellos;
    std::atomic<bool> running{true};
    std::mutex command_mutex;
    std::queue<Command> command_queue;
    std::mutex queue_mutex;
    const int COMMAND_TIMEOUT_MS = 500;
    bool busy{false};  // Add this with the other member variables

    // Helper methods
    bool allDronesReady() {
        return std::all_of(tellos.begin(), tellos.end(), 
            [](const TelloDevice& t) { return t.state == TelloDevice::CommandState::IDLE; });
    }

    // Modified processResponses() to use direct socket communication
    void processResponses() {
        for (auto& tello : tellos) {
            auto response = tello.receiveResponse();
            if (response) {
                std::cout << "Tello " << tello.ip << ": " << *response << std::endl;
                
                if (*response == "ok") {
                    tello.command_acked = true;
                    if (tello.state == TelloDevice::CommandState::WAITING_ACK) {
                        tello.state = TelloDevice::CommandState::EXECUTING;
                    }
                } else if (*response == "error") {
                    tello.state = TelloDevice::CommandState::ERROR;
                    std::cerr << "Error from Tello " << tello.ip << std::endl;
                }
                
                // Handle command completion
                if (tello.state == TelloDevice::CommandState::EXECUTING) {
                    if (tello.current_command.find("rc") == 0) {
                        if (std::chrono::steady_clock::now() - tello.last_command_time > 
                            std::chrono::milliseconds(100)) {
                            tello.state = TelloDevice::CommandState::COMPLETED;
                        }
                    } else {
                        tello.state = TelloDevice::CommandState::COMPLETED;
                    }
                }
            }
            
            // Check timeouts
            if (tello.state == TelloDevice::CommandState::WAITING_ACK) {
                if (std::chrono::steady_clock::now() - tello.last_command_time > 
                    std::chrono::milliseconds(COMMAND_TIMEOUT_MS)) {
                    std::cerr << "Command timeout for Tello " << tello.ip << std::endl;
                    tello.state = TelloDevice::CommandState::ERROR;
                }
            }
        }
    }

    void executeNextCommand() {
        std::lock_guard<std::mutex> lock(queue_mutex);
        if (command_queue.empty() || !allDronesReady()) {
            return;
        }

        auto& cmd = command_queue.front();
        bool success = true;

        for (auto& tello : tellos) {
            if (tello.error_count >= tello.MAX_ERRORS) {
                std::cout << "[REINIT] Too many errors for " << tello.ip 
                        << ", reinitializing..." << std::endl;
                tello.initializeSockets();
                tello.error_count = 0;
            }

            tello.current_command = cmd.cmd;
            tello.last_command_time = std::chrono::steady_clock::now();
            tello.command_acked = false;
            
            if (!tello.sendCommand(cmd.cmd)) {
                success = false;
                continue;
            }
            
            tello.state = TelloDevice::CommandState::WAITING_ACK;
        }

        if (!success && cmd.requires_sync) {
            std::cout << "[RETRY] Command failed, will retry: " << cmd.cmd << std::endl;
        } else {
            command_queue.pop();
        }
    }


public:
    // Send reboot command directly to a Tello drone
    bool rebootDrone(const std::string& ip, int port = 8889) {
        int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if (sock < 0) {
            std::cerr << "Failed to create socket for reboot: " << strerror(errno) << std::endl;
            return false;
        }

        // Set up address
        struct sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = inet_addr(ip.c_str());
        addr.sin_port = htons(port);

        // Send reboot command
        std::string cmd = "reboot";
        ssize_t sent = sendto(sock, cmd.c_str(), cmd.length(), 0, 
                          (struct sockaddr*)&addr, sizeof(addr));
        
        close(sock);
        
        if (sent < 0) {
            std::cerr << "Failed to send reboot to " << ip << ": " << strerror(errno) << std::endl;
            return false;
        }
        
        std::cout << "Sent reboot command to " << ip << std::endl;
        return true;
    }

    // Check if drone is responding to ping
    bool pingDrone(const std::string& ip) {
        std::string cmd = "ping -c 1 -W 1 " + ip + " > /dev/null 2>&1";
        return system(cmd.c_str()) == 0;
    }

    bool initialize() {
        std::vector<std::string> tello_ips = {
            "192.168.1.103",
            "192.168.1.104",
            "192.168.1.106",
            "192.168.1.107",
            "192.168.1.108"
        };

        std::cout << "Rebooting all Tello drones for clean start..." << std::endl;
        
        // First reboot all drones
        for (const auto& ip : tello_ips) {
            rebootDrone(ip);
        }
        
        // Wait for all drones to reboot (5-10 seconds per drone)
        std::cout << "Waiting for drones to complete reboot..." << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(10));
        
        // Additional delay to ensure Wi-Fi reconnection
        std::cout << "Waiting for drones to reconnect to Wi-Fi..." << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(10));
        
        // Check which drones are online
        std::cout << "Verifying drones are online..." << std::endl;
        for (const auto& ip : tello_ips) {
            if (pingDrone(ip)) {
                std::cout << "Drone at " << ip << " is online" << std::endl;
            } else {
                std::cout << "Drone at " << ip << " is not responding to ping" << std::endl;
            }
        }
        
        std::cout << "Attempting to connect to " << tello_ips.size() << " Tello drones..." << std::endl;

        for (size_t i = 0; i < tello_ips.size(); ++i) {
            const auto& ip = tello_ips[i];
            TelloDevice tello_device;
            tello_device.ip = ip;
            tello_device.local_port = 9000 + i;

            std::cout << "Initializing connection to Tello at " << ip 
                    << " using local port " << tello_device.local_port << std::endl;

            if (!tello_device.initializeSockets()) {
                std::cerr << "Failed to initialize sockets for Tello at " << ip << std::endl;
                continue;
            }

            // Test connection with multiple retries
            bool connected = false;
            for (int attempt = 0; attempt < 3 && !connected; attempt++) {
                if (attempt > 0) {
                    std::cout << "Retry attempt " << attempt + 1 << " for " << ip << std::endl;
                    std::this_thread::sleep_for(std::chrono::seconds(1));
                }

                if (!tello_device.sendCommand("command")) {
                    std::cerr << "Failed to send command to " << ip << std::endl;
                    continue;
                }

                // Wait for response
                auto response = tello_device.receiveResponse();
                if (response && *response == "ok") {
                    connected = true;
                    std::cout << "Successfully connected to " << ip << std::endl;
                    break;
                }
            }

            if (connected) {
                tellos.push_back(std::move(tello_device));
            } else {
                std::cerr << "Failed to connect to " << ip << " after all attempts" << std::endl;
            }
        }

        std::cout << "Connected to " << tellos.size() << " out of " << tello_ips.size() << " drones" << std::endl;
        return !tellos.empty();
    }

    void sendCommandToAll(const std::string& command, bool requires_sync = true) {
        std::lock_guard<std::mutex> lock(queue_mutex);
        command_queue.push({command, requires_sync, std::chrono::steady_clock::now()});
    }

    void waitForAllResponses(const std::string& command, int timeout_ms = 500) {
        if (command.find("rc") == 0) return;  // Don't wait for RC commands
        
        auto start = std::chrono::steady_clock::now();
        while (std::chrono::steady_clock::now() - start < std::chrono::milliseconds(timeout_ms)) {
            bool all_responded = true;
            for (const auto& tello : tellos) {
                if (!tello.command_acked) {
                    all_responded = false;
                    break;
                }
            }
            if (all_responded) break;
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }

    // Cleanup function to safely land and shutdown all drones
    void cleanup() {
        std::cout << "Performing cleanup..." << std::endl;
        
        // Land all flying drones
        for (auto& tello : tellos) {
            std::cout << "Sending land command to " << tello.ip << std::endl;
            tello.sendCommand("land");
            
            // Wait a bit for landing to complete
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            
            // Get response to ensure command was received
            auto response = tello.receiveResponse();
            if (response) {
                std::cout << "Response from " << tello.ip << ": " << *response << std::endl;
            }
        }
        
        // Wait for landing to complete
        std::this_thread::sleep_for(std::chrono::seconds(3));
        
        // Optionally reboot all drones for clean state
        std::cout << "Rebooting all drones for clean slate..." << std::endl;
        for (auto& tello : tellos) {
            rebootDrone(tello.ip);
        }
        
        // Close all sockets
        for (auto& tello : tellos) {
            if (tello.command_socket >= 0) {
                close(tello.command_socket);
                tello.command_socket = -1;
            }
        }
        
        std::cout << "Cleanup complete" << std::endl;
    }
    
    // Helper method to handle special keys
    char getSpecialKey(KeyboardInput& keyboard) {
        char c = keyboard.getKey();
        if (c == 27) {  // ESC character
            char next = keyboard.getKey();
            if (next == '[') {  // Arrow keys are ESC [ A/B/C/D
                char arrow = keyboard.getKey();
                return (arrow == 'A') ? KEY_UP :
                       (arrow == 'B') ? KEY_DOWN :
                       (arrow == 'C') ? KEY_RIGHT :
                       (arrow == 'D') ? KEY_LEFT : -1;
            }
        }
        return c;
    }

    // Constants for special keys
    static constexpr char KEY_UP = 128;
    static constexpr char KEY_DOWN = 129;
    static constexpr char KEY_LEFT = 130;
    static constexpr char KEY_RIGHT = 131;

    void run() {
        // Register signal handlers for clean shutdown
        g_controller = this;
        signal(SIGINT, signalHandler);
        signal(SIGTERM, signalHandler);
        
        KeyboardInput keyboard;
        const int speed = 50;
        
        std::cout << "Controller running. Controls:\n";
        std::cout << "  't' - takeoff\n";
        std::cout << "  'l' - land\n";
        std::cout << "  'q' - quit\n";
        std::cout << "  'w/a/s/d' - forward/left/backward/right\n";
        std::cout << "  'up/down' - increase/decrease altitude\n";
        std::cout << "  'left/right' - yaw counter-clockwise/clockwise\n";
        std::cout << "  'space' - hover in place (stop all movement)\n";
        std::cout << "  'backspace' - emergency stop (cuts engines)\n";
        
        // Main control loop
        while (running) {
            char key = getSpecialKey(keyboard);
            if (key != -1) {
                std::string command;
                bool requires_sync = true;
                bool is_rc_command = false;
                
                // RC values: left-right, forward-backward, up-down, yaw
                int lr = 0, fb = 0, ud = 0, yaw = 0;
                
                switch (key) {
                    case 'q':
                        cleanup();
                        running = false;
                        break;
                    case 't': 
                        if (!busy) {
                            command = "takeoff"; 
                            busy = true;
                        }
                        break;
                    case 'l': 
                        if (!busy) {
                            command = "land"; 
                            busy = true;
                        }
                        break;
                    // RC commands - WASD for lateral movement
                    case 'w': // Forward
                        fb = speed;
                        is_rc_command = true;
                        break;
                    case 's': // Backward
                        fb = -speed;
                        is_rc_command = true;
                        break;
                    case 'a': // Left 
                        lr = -speed;
                        is_rc_command = true;
                        break;
                    case 'd': // Right
                        lr = speed;
                        is_rc_command = true;
                        break;
                    // Arrow keys for altitude and yaw
                    case KEY_UP: // Up
                        ud = speed;
                        is_rc_command = true;
                        break;
                    case KEY_DOWN: // Down
                        ud = -speed;
                        is_rc_command = true;
                        break;
                    case KEY_LEFT: // Rotate left (counterclockwise)
                        yaw = -speed;
                        is_rc_command = true;
                        break;
                    case KEY_RIGHT: // Rotate right (clockwise)
                        yaw = speed;
                        is_rc_command = true;
                        break;
                    // Space to hover in place (stop all movement)
                    case ' ':
                        command = "rc 0 0 0 0";
                        is_rc_command = true;
                        break;
                    // Backspace for emergency stop (cuts engines)
                    case 127: // ASCII code for backspace
                        if (!busy) {
                            command = "emergency";
                            busy = true;
                        }
                        break;
                }
                
                // Build RC command if any movement values are set
                if (is_rc_command) {
                    command = "rc " + std::to_string(lr) + " " + 
                                      std::to_string(fb) + " " + 
                                      std::to_string(ud) + " " + 
                                      std::to_string(yaw);
                    requires_sync = false;
                }
                
                if (!command.empty()) {
                    bool send_success = true;
                    for (auto& tello : tellos) {
                        if (is_rc_command && !tello.shouldSendRCCommand()) {
                            continue;  // Skip if too soon for RC command
                        }
                        if (!tello.sendCommand(command)) {
                            send_success = false;
                        }
                    }

                    if (requires_sync && send_success) {
                        waitForAllResponses(command);
                    }
                }
            }

            // Process responses
            for (auto& tello : tellos) {
                auto response = tello.receiveResponse();
                if (response) {
                    if (*response == "ok") {
                        busy = false;
                    }
                }
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
};

// Implement the signal handler after the class definition
void signalHandler(int signal) {
    if (g_controller) {
        std::cout << "\nShutting down drones safely..." << std::endl;
        g_controller->cleanup();
    }
    exit(signal);
}

int main() {
    MultiTelloController controller;
    if (!controller.initialize()) {
        std::cerr << "Failed to initialize Tello connections!" << std::endl;
        return 1;
    }
    
    try {
        controller.run();
    } catch (const std::exception& e) {
        std::cerr << "Error during execution: " << e.what() << std::endl;
        controller.cleanup();
        return 1;
    }
    
    // Make sure we clean up properly on normal exit
    controller.cleanup();
    return 0;
}