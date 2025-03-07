// src/drift_calibrator.cpp
#include "FileHandler.hpp"
#include "file_writer.hpp" // Add the new file writer
#include <iostream>
#include <vector>
#include <chrono>
#include <thread>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <poll.h>
#include <cstring>
#include <cmath>
#include <mutex>
#include <atomic>
#include <condition_variable>
#include <queue>
#include <unordered_map>
#include <memory>
#include <iomanip>
#include <sstream>
#include <sys/wait.h>
#include <signal.h>

// Constants for special keys
const char KEY_UP = 128;
const char KEY_DOWN = 129;
const char KEY_LEFT = 130;
const char KEY_RIGHT = 131;

// Define a structure to hold command and response data
struct CommandData {
    std::string ip;
    std::string command;
    std::string response;
    bool completed{false};
    std::chrono::steady_clock::time_point timestamp;
};

class DroneManager {
private:
    // Thread-safe command queue
    class CommandQueue {
    private:
        std::queue<std::shared_ptr<CommandData>> queue;
        std::mutex mtx;
        std::condition_variable cv;
        std::atomic<bool> stop{false};

    public:
        void push(std::shared_ptr<CommandData> cmd) {
            std::lock_guard<std::mutex> lock(mtx);
            queue.push(cmd);
            cv.notify_one();
        }

        std::shared_ptr<CommandData> pop() {
            std::unique_lock<std::mutex> lock(mtx);
            cv.wait_for(lock, std::chrono::milliseconds(100), [this] { 
                return !queue.empty() || stop.load(); 
            });
            if (queue.empty() || stop.load()) return nullptr;
            
            auto cmd = queue.front();
            queue.pop();
            return cmd;
        }

        void stopQueue() {
            stop.store(true);
            cv.notify_all();
        }

        bool isEmpty() {
            std::lock_guard<std::mutex> lock(mtx);
            return queue.empty();
        }
    };

    // Keyboard handling with non-blocking reads
    class KeyboardInput {
    private:
        struct termios orig_termios;
        bool initialized{false};
        int original_flags;
        
    public:
        KeyboardInput() {
            // Save original terminal settings and flags
            if (tcgetattr(STDIN_FILENO, &orig_termios) == 0) {
                original_flags = fcntl(STDIN_FILENO, F_GETFL);
                
                struct termios raw = orig_termios;
                raw.c_lflag &= ~(ICANON | ECHO);
                raw.c_cc[VMIN] = 0;
                raw.c_cc[VTIME] = 0;
                if (tcsetattr(STDIN_FILENO, TCSAFLUSH, &raw) == 0) {
                    fcntl(STDIN_FILENO, F_SETFL, original_flags | O_NONBLOCK);
                    initialized = true;
                }
            }
        }
        
        ~KeyboardInput() {
            if (initialized) {
                // Restore original terminal settings
                tcsetattr(STDIN_FILENO, TCSAFLUSH, &orig_termios);
                // Restore original flags
                fcntl(STDIN_FILENO, F_SETFL, original_flags);
                
                // Extra safety - make sure terminal is in a usable state
                struct termios tty;
                tcgetattr(STDIN_FILENO, &tty);
                tty.c_lflag |= (ICANON | ECHO); // Re-enable canonical mode and echo
                tcsetattr(STDIN_FILENO, TCSAFLUSH, &tty);
                
                // Clear screen and reset cursor for clean exit
                std::cout << "\033[2J\033[1;1H" << std::flush;
            }
        }
        
        int getKey() {
            // Use select to wait for input with a timeout
            fd_set readfds;
            struct timeval timeout;
            
            FD_ZERO(&readfds);
            FD_SET(STDIN_FILENO, &readfds);
            
            timeout.tv_sec = 0;
            timeout.tv_usec = 50000; // 50ms timeout
            
            int ready = select(STDIN_FILENO + 1, &readfds, NULL, NULL, &timeout);
            if (ready <= 0) return -1;
            
            // Now read is guaranteed not to block
            char c;
            int result = read(STDIN_FILENO, &c, 1);
            if (result <= 0) return -1;
            
            // Handle arrow keys and special key sequences
            if (c == 27) {  // Escape sequence
                fd_set escfds;
                FD_ZERO(&escfds);
                FD_SET(STDIN_FILENO, &escfds);
                
                // Very short timeout for ESC sequence
                timeout.tv_sec = 0;
                timeout.tv_usec = 20000; // 20ms
                
                if (select(STDIN_FILENO + 1, &escfds, NULL, NULL, &timeout) > 0) {
                    char next;
                    if (read(STDIN_FILENO, &next, 1) > 0 && next == '[') {
                        // We have an arrow key sequence
                        if (select(STDIN_FILENO + 1, &escfds, NULL, NULL, &timeout) > 0) {
                            char arrow;
                            if (read(STDIN_FILENO, &arrow, 1) > 0) {
                                switch (arrow) {
                                    case 'A': return KEY_UP;
                                    case 'B': return KEY_DOWN;
                                    case 'C': return KEY_RIGHT;
                                    case 'D': return KEY_LEFT;
                                }
                            }
                        }
                    }
                }
                return 27;  // Escape key
            }
            
            return c;
        }
        
        enum SpecialKeys {
            KEY_UP = 1000,
            KEY_DOWN,
            KEY_RIGHT,
            KEY_LEFT,
            KEY_ESCAPE = 27
        };
    };

    std::vector<DroneConfig> drones;
    std::unordered_map<std::string, int> drone_sockets;
    std::unordered_map<std::string, sockaddr_in> drone_addresses;
    size_t current_drone{0};
    float fine_adjust{0.1f};
    float coarse_adjust{1.0f};
    std::atomic<bool> running{true};
    std::atomic<bool> emergency_stop{false};
    std::mutex calibration_mutex;
    CommandQueue command_queue;
    std::thread command_thread;
    std::thread vrpn_thread;
    std::thread watchdog_thread;
    int command_timeout_ms{750};  // Command timeout in ms
    int flight_timeout_sec{45};   // Maximum flight time in seconds

    std::atomic<bool> has_vrpn_data{false};
    std::unordered_map<std::string, VrpnData> tracker_data;
    std::mutex tracker_mutex;
    std::optional<std::array<float, 4>> reference_orientation;  // Reference orientation quaternion for calibration

    // Initialize socket for a drone
    bool initSocket(DroneConfig& drone) {
        if (drone_sockets.count(drone.ip) > 0 && drone_sockets[drone.ip] >= 0) {
            close(drone_sockets[drone.ip]);
        }

        int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if (sock < 0) {
            std::cerr << "Failed to create socket for " << drone.ip << std::endl;
            return false;
        }

        // Set up local address
        struct sockaddr_in local_addr{};
        local_addr.sin_family = AF_INET;
        local_addr.sin_addr.s_addr = INADDR_ANY;
        local_addr.sin_port = htons(0);  // Let system choose port

        // Bind to local port
        if (bind(sock, (struct sockaddr *)&local_addr, sizeof(local_addr)) < 0) {
            std::cerr << "Failed to bind socket for " << drone.ip << ": " << strerror(errno) << std::endl;
            close(sock);
            return false;
        }

        // Set up remote address
        struct sockaddr_in remote_addr{};
        remote_addr.sin_family = AF_INET;
        remote_addr.sin_addr.s_addr = inet_addr(drone.ip.c_str());
        remote_addr.sin_port = htons(8889);

        // Set socket timeout
        struct timeval tv;
        tv.tv_sec = 0;
        tv.tv_usec = 500000;  // 500ms timeout
        setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

        drone_sockets[drone.ip] = sock;
        drone_addresses[drone.ip] = remote_addr;

        return true;
    }

    // Command processing thread
    void commandProcessor() {
        std::vector<pollfd> poll_fds;
        std::unordered_map<int, std::shared_ptr<CommandData>> active_commands;

        while (running.load()) {
            if (emergency_stop.load()) {
                // Emergency stop - land all drones
                poll_fds.clear();
                active_commands.clear();
                emergencyLandAll();
                emergency_stop.store(false);
            }

            // Process new commands from queue
            while (auto cmd = command_queue.pop()) {
                if (!cmd) break; // Queue was stopped or empty

                if (drone_sockets.count(cmd->ip) == 0 || drone_sockets[cmd->ip] < 0) {
                    cmd->response = "Socket not initialized";
                    cmd->completed = true;
                    continue;
                }

                int sock = drone_sockets[cmd->ip];
                
                // Send command
                ssize_t sent = sendto(sock, cmd->command.c_str(), cmd->command.length(), 0,
                                  (struct sockaddr *)&drone_addresses[cmd->ip], 
                                  sizeof(drone_addresses[cmd->ip]));
                
                if (sent < 0) {
                    cmd->response = "Send failed: " + std::string(strerror(errno));
                    cmd->completed = true;
                    continue;
                }

                // Add to active commands
                cmd->timestamp = std::chrono::steady_clock::now();
                active_commands[sock] = cmd;
                
                // Add socket to poll list
                pollfd pfd;
                pfd.fd = sock;
                pfd.events = POLLIN;
                poll_fds.push_back(pfd);
            }

            if (poll_fds.empty()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }

            // Poll for responses with a short timeout
            int result = poll(poll_fds.data(), poll_fds.size(), 50);
            
            if (result > 0) {
                for (size_t i = 0; i < poll_fds.size(); i++) {
                    if (poll_fds[i].revents & POLLIN) {
                        int sock = poll_fds[i].fd;
                        char buffer[1024];
                        struct sockaddr_in from_addr;
                        socklen_t from_len = sizeof(from_addr);
                        
                        ssize_t received = recvfrom(sock, buffer, sizeof(buffer)-1, 0,
                                              (struct sockaddr *)&from_addr, &from_len);
                        
                        if (received > 0) {
                            buffer[received] = '\0';
                            if (active_commands.find(sock) != active_commands.end()) {
                                active_commands[sock]->response = buffer;
                                active_commands[sock]->completed = true;
                                
                                // Remove from active commands
                                active_commands.erase(sock);
                                poll_fds.erase(poll_fds.begin() + i);
                                i--; // Adjust index after removal
                            }
                        }
                    }
                }
            }

            // Check for timeouts
            auto now = std::chrono::steady_clock::now();
            for (auto it = active_commands.begin(); it != active_commands.end(); ) {
                auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                               now - it->second->timestamp).count();
                
                if (elapsed > command_timeout_ms) { // Command timeout
                    it->second->response = "Timeout";
                    it->second->completed = true;
                    
                    // Remove from poll_fds
                    for (size_t i = 0; i < poll_fds.size(); i++) {
                        if (poll_fds[i].fd == it->first) {
                            poll_fds.erase(poll_fds.begin() + i);
                            break;
                        }
                    }
                    
                    it = active_commands.erase(it);
                } else {
                    ++it;
                }
            }

            // Small delay to prevent CPU hogging
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }

    // Flight safety watchdog thread - forces landing after timeout
    void flightWatchdog() {
        // Track each drone's flight time
        std::unordered_map<std::string, std::chrono::steady_clock::time_point> flight_start_times;
        
        while (running.load()) {
            for (auto& drone : drones) {
                if (drone.is_flying) {
                    // If newly flying, record start time
                    if (flight_start_times.find(drone.ip) == flight_start_times.end()) {
                        flight_start_times[drone.ip] = std::chrono::steady_clock::now();
                    } else {
                        // Check flight duration
                        auto now = std::chrono::steady_clock::now();
                        auto duration = std::chrono::duration_cast<std::chrono::seconds>(
                                       now - flight_start_times[drone.ip]).count();
                        
                        if (duration > flight_timeout_sec) {
                            std::cout << "Safety timeout: Landing drone " << drone.ip 
                                      << " after " << duration << " seconds of flight" << std::endl;
                            // Force land the drone
                            auto cmd = sendCommand(drone.ip, "land");
                            drone.is_flying = false;
                            flight_start_times.erase(drone.ip);
                        }
                    }
                } else {
                    // Reset timer if drone landed
                    flight_start_times.erase(drone.ip);
                }
            }
            
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }

    // Connect to VRPN server
    bool connectVrpn() {
        // Socket-based implementation
        int vrpn_socket = socket(AF_INET, SOCK_STREAM, 0);
        if (vrpn_socket < 0) {
            std::cerr << "Failed to create VRPN socket" << std::endl;
            return false;
        }

        struct sockaddr_in server_addr{};
        server_addr.sin_family = AF_INET;
        server_addr.sin_addr.s_addr = inet_addr("192.168.1.100");
        server_addr.sin_port = htons(3883);

        if (connect(vrpn_socket, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
            std::cerr << "Failed to connect to VRPN server: " << strerror(errno) << std::endl;
            close(vrpn_socket);
            return false;
        }

        // Set socket to non-blocking
        int flags = fcntl(vrpn_socket, F_GETFL, 0);
        fcntl(vrpn_socket, F_SETFL, flags | O_NONBLOCK);

        // Start VRPN thread
        vrpn_thread = std::thread([this, vrpn_socket]() {
            char buffer[4096];
            while (running.load()) {
                // Non-blocking read
                ssize_t received = recv(vrpn_socket, buffer, sizeof(buffer)-1, 0);
                if (received > 0) {
                    buffer[received] = '\0';
                    
                    std::lock_guard<std::mutex> lock(tracker_mutex);
                    // Parse tracker data from buffer
                    std::string data(buffer);
                    std::istringstream stream(data);
                    std::string line;
                    
                    while (std::getline(stream, line)) {
                        std::istringstream linestream(line);
                        std::string name;
                        float x, y, z, qw, qx, qy, qz;
                        
                        std::getline(linestream, name, ',');
                        if (linestream >> x >> y >> z >> qw >> qx >> qy >> qz) {
                            VrpnData vrpn_data;
                            vrpn_data.position = {x, y, z};
                            vrpn_data.rotation = {qw, qx, qy, qz};
                            tracker_data[name] = vrpn_data;
                        }
                    }
                    
                    has_vrpn_data.store(true);
                }
                
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
            
            close(vrpn_socket);
        });

        return true;
    }

    // Send command with async response
    std::shared_ptr<CommandData> sendCommand(const std::string& ip, const std::string& cmd) {
        auto command_data = std::make_shared<CommandData>();
        command_data->ip = ip;
        command_data->command = cmd;
        command_data->completed = false;
        
        // Add to command queue
        command_queue.push(command_data);
        
        return command_data;
    }

    // Send command and wait for response (blocking)
    std::string sendCommandBlocking(const std::string& ip, const std::string& cmd, int timeout_ms = 2000) {
        auto command = sendCommand(ip, cmd);
        
        // Wait for response with timeout
        auto start = std::chrono::steady_clock::now();
        while (!command->completed) {
            auto now = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count();
            if (elapsed > timeout_ms) {
                return "Timeout";
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        
        return command->response;
    }

    // Force land all drones immediately (emergency function)
    void emergencyLandAll() {
        std::cout << "\n*** EMERGENCY STOP - LANDING ALL DRONES ***\n" << std::endl;
        for (auto& drone : drones) {
            if (drone.is_flying) {
                sendCommandBlocking(drone.ip, "land");
                drone.is_flying = false;
            }
        }
    }

    // Apply bias to drone during drift correction
    void applyBias(DroneConfig& drone) {
        if (!drone.is_flying) return;

        // Use values directly (Tello expects values from -100 to 100)
        int pitch = static_cast<int>(drone.pitch_bias);
        int roll = static_cast<int>(drone.roll_bias);
        int yaw = static_cast<int>(drone.yaw_bias);
        
        // Clamp to safe values (-100 to 100)
        pitch = std::max(-100, std::min(pitch, 100));
        roll = std::max(-100, std::min(roll, 100));
        yaw = std::max(-100, std::min(yaw, 100));

        // Throttle value (0 for hovering)
        int throttle = 0;

        std::string cmd = "rc " + std::to_string(roll) + " " +
                        std::to_string(pitch) + " " + 
                        std::to_string(throttle) + " " +
                        std::to_string(yaw);
                        
        sendCommand(drone.ip, cmd);
        // Non-blocking, let the command processor handle response
    }

    // Display drift calibration status
    void displayStatus() {
        // Use ANSI escape sequence for clearing screen instead of system("clear")
        std::cout << "\033[2J\033[1;1H";  // Clear screen and move cursor to top
        std::cout.flush();
        
        std::cout << "Drift Calibration Tool\n";
        std::cout << "=====================\n\n";
        std::cout << "Controls:\n";
        std::cout << "W/A/S/D: Coarse pitch/roll adjustment (±" << coarse_adjust << ")\n";
        std::cout << "I/J/K/L: Fine pitch/roll adjustment (±" << fine_adjust << ")\n";
        std::cout << "Q/E: Yaw adjustment (±" << coarse_adjust << ")\n";
        std::cout << "U/O: Fine yaw adjustment (±" << fine_adjust << ")\n";
        std::cout << "T: Take off\n";
        std::cout << "Space: Land (current drone)\n";
        std::cout << "N: Next drone (lands current drone first)\n";
        std::cout << "R: Reset current drone values\n";
        std::cout << "S: Save and quit\n";
        std::cout << "Esc: Quit without saving\n\n";

        if (current_drone < drones.size()) {
            auto& drone = drones[current_drone];
            std::cout << "Current Drone: " << drone.ip << " (" << drone.mac << ")\n";
            std::cout << "Status: " << (drone.is_flying ? "Flying" : "Landed") << "\n\n";
            
            std::cout << "Bias Values:\n";
            std::cout << "Pitch: " << std::fixed << std::setprecision(1) << drone.pitch_bias << "\n";
            std::cout << "Roll: " << std::fixed << std::setprecision(1) << drone.roll_bias << "\n";
            std::cout << "Yaw: " << std::fixed << std::setprecision(1) << drone.yaw_bias << "\n";
            
            if (drone.tracker_id.empty()) {
                std::cout << "\nNo tracker assigned to this drone\n";
            } else {
                std::cout << "\nTracker: " << drone.tracker_id << "\n";
                
                // Display tracker data if available
                std::lock_guard<std::mutex> lock(tracker_mutex);
                if (tracker_data.find(drone.tracker_id) != tracker_data.end()) {
                    auto& data = tracker_data[drone.tracker_id];
                    std::cout << "Position: [" << std::fixed << std::setprecision(2) 
                              << data.position[0] << ", " 
                              << data.position[1] << ", " << data.position[2] << "]\n";
                    std::cout << "Rotation: [" << std::fixed << std::setprecision(3)
                              << data.rotation[0] << ", " 
                              << data.rotation[1] << ", " << data.rotation[2] << ", "
                              << data.rotation[3] << "]\n";
                }
            }
        }
    }

    // Helper method to handle special keys for flight controls
    char getSpecialKey() {
        // Set terminal to non-canonical mode
        struct termios old_tio, new_tio;
        tcgetattr(STDIN_FILENO, &old_tio);
        new_tio = old_tio;
        new_tio.c_lflag &= ~(ICANON | ECHO);
        new_tio.c_cc[VMIN] = 0;
        new_tio.c_cc[VTIME] = 0;
        tcsetattr(STDIN_FILENO, TCSANOW, &new_tio);
        fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);
        
        // Read key with proper timeout to prevent CPU hogging
        char c;
        fd_set readfds;
        struct timeval timeout;
        
        FD_ZERO(&readfds);
        FD_SET(STDIN_FILENO, &readfds);
        
        timeout.tv_sec = 0;
        timeout.tv_usec = 50000; // 50ms timeout
        
        int ready = select(STDIN_FILENO + 1, &readfds, NULL, NULL, &timeout);
        
        if (ready <= 0) {
            tcsetattr(STDIN_FILENO, TCSANOW, &old_tio);
            return -1;
        }
        
        int result = read(STDIN_FILENO, &c, 1);
        
        if (result <= 0) {
            tcsetattr(STDIN_FILENO, TCSANOW, &old_tio);
            return -1;
        }
        
        if (c == 27) {  // ESC character
            fd_set escfds;
            FD_ZERO(&escfds);
            FD_SET(STDIN_FILENO, &escfds);
            
            timeout.tv_sec = 0;
            timeout.tv_usec = 50000; // 50ms timeout for ESC sequence
            
            // Check if there's more input (for arrow keys)
            if (select(STDIN_FILENO + 1, &escfds, NULL, NULL, &timeout) > 0) {
                char next = -1;
                read(STDIN_FILENO, &next, 1);
                
                if (next == '[') {  // Arrow keys are ESC [ A/B/C/D
                    char arrow = -1;
                    if (read(STDIN_FILENO, &arrow, 1) > 0) {
                        // Restore terminal settings
                        tcsetattr(STDIN_FILENO, TCSANOW, &old_tio);
                        return (arrow == 'A') ? KEY_UP :
                               (arrow == 'B') ? KEY_DOWN :
                               (arrow == 'C') ? KEY_RIGHT :
                               (arrow == 'D') ? KEY_LEFT : -1;
                    }
                }
            }
            
            // If we reach here, it was a standalone ESC key
            tcsetattr(STDIN_FILENO, TCSANOW, &old_tio);
            return 27;
        }
        
        // Restore terminal settings
        tcsetattr(STDIN_FILENO, TCSANOW, &old_tio);
        return c;
    }
    
    // Launch VRPN visualization in a separate process
    pid_t launchVrpnViz() {
        pid_t pid = fork();
        
        if (pid == 0) {
            // Child process - redirect stdout and stderr to /dev/null to avoid console spam
            int null_fd = open("/dev/null", O_WRONLY);
            dup2(null_fd, STDOUT_FILENO);
            dup2(null_fd, STDERR_FILENO);
            close(null_fd);
            
            const char* vrpn_viz_path = "/home/finn/squawkblock/squawkblock/vrpn/build/vrpn_viz";
            execl(vrpn_viz_path, vrpn_viz_path, NULL);
            
            // If exec fails
            exit(1);
        }
        
        // Parent process - return child PID
        return pid;
    }
    
    // Map a single drone to a tracker with interactive flight and visualizer
    bool mapSingleDrone() {
        // Menu to select which drone to map
        std::cout << "Select a drone to map:\n";
        std::vector<size_t> connected_indices;
        
        for (size_t i = 0; i < drones.size(); i++) {
            if (drones[i].is_connected) {
                connected_indices.push_back(i);
                std::string status = drones[i].tracker_id.empty() ? "UNMAPPED" : "Mapped to " + drones[i].tracker_id;
                std::cout << connected_indices.size() << ": " << drones[i].ip 
                          << " (" << drones[i].mac << ") - " << status << "\n";
            }
        }
        
        if (connected_indices.empty()) {
            std::cout << "No connected drones found. Please scan for drones first.\n";
            std::cout << "\nPress any key to return to menu...";
            std::cin.get();
            return false;
        }
        
        std::cout << "0: Return to main menu\n";
        std::cout << "Enter selection: ";
        
        int selection = 0;
        std::cin >> selection;
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');  // Clear input buffer
        
        if (selection <= 0 || selection > static_cast<int>(connected_indices.size())) {
            return false; // Return to main menu
        }
        
        // Get the selected drone
        auto& drone = drones[connected_indices[selection - 1]];
        
        if (!drone.tracker_id.empty()) {
            std::cout << "Drone " << drone.ip << " already mapped to " << drone.tracker_id << "\n";
            std::cout << "Do you want to remap? (y/n): ";
            char choice;
            std::cin >> choice;
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');  // Clear input buffer
            
            if (choice != 'y' && choice != 'Y') {
                return false; // Return to main menu
            }
        }
        
        // Check if the VRPN connection is active
        if (!has_vrpn_data.load()) {
            std::cout << "ERROR: No VRPN tracking data available. Cannot map drones.\n"
                     << "Make sure the OptiTrack system is running and the VRPN server is connected.\n";
            std::cout << "\nPress any key to return to menu...";
            std::cin.get();
            return false;
        }
        
        // Launch VRPN visualizer
        std::cout << "Launching OptiTrack visualizer...\n";
        pid_t viz_pid = launchVrpnViz();
        std::this_thread::sleep_for(std::chrono::seconds(2)); // Wait for visualizer to start
        
        // Reboot the selected drone for a fresh state
        std::cout << "Rebooting drone for a fresh state...\n";
        
        // Create a direct reboot socket
        int reboot_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if (reboot_sock >= 0) {
            // Set up address
            struct sockaddr_in addr{};
            addr.sin_family = AF_INET;
            addr.sin_addr.s_addr = inet_addr(drone.ip.c_str());
            addr.sin_port = htons(8889);
            
            // Send reboot command
            std::string cmd = "reboot";
            ssize_t sent = sendto(reboot_sock, cmd.c_str(), cmd.length(), 0, 
                              (struct sockaddr*)&addr, sizeof(addr));
            
            close(reboot_sock);
            
            if (sent > 0) {
                std::cout << "Sent reboot command to " << drone.ip << std::endl;
                // Wait for drone to reboot
                std::cout << "Waiting for drone to reboot (8 seconds)...\n";
                std::this_thread::sleep_for(std::chrono::seconds(8));
            }
        }
            
            std::cout << "\nMapping drone " << drone.ip << "...\n";
            std::cout << "Flashing LED to identify physical drone\n";
            
            sendCommandBlocking(drone.ip, "led 255 0 0");
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            sendCommandBlocking(drone.ip, "led 0 0 0");
            
            std::cout << "Clear area around drone and press Enter to take off...";
            std::cin.get();
            
            // Send command mode first like swarm.cpp does
            std::cout << "Sending command mode...\n";
            std::string cmd_response = sendCommandBlocking(drone.ip, "command");
            if (cmd_response != "ok") {
                std::cout << "WARNING: Command mode response: " << cmd_response << ", trying anyway...\n";
            }
            
            // Add delay before takeoff
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            
            // Take off drone using direct socket communication, not relying on response
            std::cout << "Taking off drone...\n";
            
            // Create a direct socket for takeoff
            int direct_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
            if (direct_sock < 0) {
                std::cerr << "Failed to create socket for takeoff: " << strerror(errno) << std::endl;
                return false;
            }
            
            // Set up address
            struct sockaddr_in addr{};
            addr.sin_family = AF_INET;
            addr.sin_addr.s_addr = inet_addr(drone.ip.c_str());
            addr.sin_port = htons(8889);
            
            // Send takeoff command directly
            std::string takeoff_cmd = "takeoff";
            sendto(direct_sock, takeoff_cmd.c_str(), takeoff_cmd.length(), 0, 
                   (struct sockaddr*)&addr, sizeof(addr));
            
            close(direct_sock);
            
            // Wait for takeoff to complete
            std::cout << "Takeoff command sent, waiting for drone to take off...\n";
            std::this_thread::sleep_for(std::chrono::seconds(5));  // Wait for takeoff to complete
            
            std::cout << "Press Enter to continue with flight control...";
            std::cin.get();  // Just wait for Enter, no confirmation needed
            
            drone.is_flying = true;
            
            // Create a position log file for this drone using our reliable file writer
            std::string log_filename = "positions_" + drone.ip + ".txt";
            std::cout << "Creating position log at: " << log_filename << std::endl;
            
            // Use our reliable file writer to create the position log
            std::ofstream position_log = FileWriter::createPositionLog(drone.ip);
            
            if (position_log.is_open()) {
                // Write a test entry to confirm file is working
                bool test_write = FileWriter::logPosition(position_log, "test_entry", 0, 0, 0);
                
                if (test_write) {
                    std::cout << "✓ Position logging initialized and test write successful" << std::endl;
                } else {
                    std::cerr << "WARNING: Position log test write failed" << std::endl;
                }
            } else {
                std::cerr << "ERROR: Failed to create position log file" << std::endl;
            }
            
            // Interactive flight control
            system("clear");
            std::cout << "\n--- Flight Controls ---\n";
            std::cout << "  'w/a/s/d' - forward/left/backward/right\n";
            std::cout << "  'up/down arrows' - up/down\n";
            std::cout << "  'left/right arrows' - rotate left/right\n";
            std::cout << "  'space' - hover in place\n";
            std::cout << "  'l' - land and confirm tracking\n";
            std::cout << "  'q' - land and skip this drone\n";
            std::cout << "\nObserve the visualizer and identify which marker is moving.\n";
            std::cout << "Fly the drone around to confirm the correct tracker.\n";
            std::cout << "\nPosition logging enabled: " << log_filename << "\n";
            
            bool flying = true;
            bool confirm_tracker = false;
            const int speed = 20; // Slower speed for mapping
            std::string moved_tracker;
            
            // Capture all trackers that are currently active
            std::unordered_map<std::string, bool> active_trackers;
            {
                std::lock_guard<std::mutex> lock(tracker_mutex);
                for (const auto& [tracker_id, data] : tracker_data) {
                    active_trackers[tracker_id] = true;
                }
            }
            
            // Previous positions to calculate movement
            std::unordered_map<std::string, std::array<float, 3>> last_positions;
            {
                std::lock_guard<std::mutex> lock(tracker_mutex);
                for (const auto& [tracker_id, data] : tracker_data) {
                    last_positions[tracker_id] = data.position;
                }
            }
            
            // Save original terminal settings at the beginning of the flight control
            struct termios orig_flight_tty;
            tcgetattr(STDIN_FILENO, &orig_flight_tty);
            
            while (flying) {
                // Get keyboard input
                char key = getSpecialKey();
                int lr = 0, fb = 0, ud = 0, yaw = 0;
                
                if (key != -1) {
                    switch (key) {
                        case 'q': // Land and skip
                            std::cout << "Landing drone, tracker mapping skipped.\n";
                            flying = false;
                            break;
                        case 'l': // Land and confirm
                            std::cout << "Landing drone and confirming tracker mapping.\n";
                            std::cout.flush(); // Ensure output is displayed

                            // Make sure we don't exit the loop too quickly
                            std::this_thread::sleep_for(std::chrono::seconds(1));
                            
                            // Set flags
                            flying = false;
                            confirm_tracker = true;
                            break;
                        case 'w': // Forward
                            fb = speed;
                            break;
                        case 's': // Backward
                            fb = -speed;
                            break;
                        case 'a': // Left
                            lr = -speed;
                            break;
                        case 'd': // Right
                            lr = speed;
                            break;
                        case KEY_UP: // Up
                            ud = speed;
                            break;
                        case KEY_DOWN: // Down
                            ud = -speed;
                            break;
                        case KEY_LEFT: // Rotate left
                            yaw = -speed;
                            break;
                        case KEY_RIGHT: // Rotate right
                            yaw = speed;
                            break;
                        case ' ': // Hover
                            lr = fb = ud = yaw = 0;
                            break;
                    }
                    
                    // Send RC command directly via socket for reliable control
                    std::string rc_cmd = "rc " + std::to_string(lr) + " " + 
                                         std::to_string(fb) + " " + 
                                         std::to_string(ud) + " " + 
                                         std::to_string(yaw);
                    
                    // Increase speed for more visible movement
                    const int high_speed = 50; // Use much higher speed for mapping
                    
                    // Apply high speed to any non-zero movement to make it more visible
                    if (lr != 0) lr = (lr > 0) ? high_speed : -high_speed;
                    if (fb != 0) fb = (fb > 0) ? high_speed : -high_speed;
                    if (ud != 0) ud = (ud > 0) ? high_speed : -high_speed;
                    if (yaw != 0) yaw = (yaw > 0) ? high_speed : -high_speed;
                    
                    // Recreate command with higher speed values
                    rc_cmd = "rc " + std::to_string(lr) + " " + 
                            std::to_string(fb) + " " + 
                            std::to_string(ud) + " " + 
                            std::to_string(yaw);
                    
                    // Create a direct socket for RC command
                    int rc_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
                    if (rc_sock >= 0) {
                        // Set up address
                        struct sockaddr_in addr{};
                        addr.sin_family = AF_INET;
                        addr.sin_addr.s_addr = inet_addr(drone.ip.c_str());
                        addr.sin_port = htons(8889);
                        
                        // Send RC command directly
                        sendto(rc_sock, rc_cmd.c_str(), rc_cmd.length(), 0, 
                               (struct sockaddr*)&addr, sizeof(addr));
                        
                        close(rc_sock);
                    } else {
                        // Fall back to queue-based command if socket creation fails
                        sendCommand(drone.ip, rc_cmd);
                    }
                }
                
                // Check for tracker movement to identify which one is moving
                std::unordered_map<std::string, float> tracker_movement;
                
                {
                    std::lock_guard<std::mutex> lock(tracker_mutex);
                    
                    // Calculate movement for each tracker
                    for (const auto& [tracker_id, data] : tracker_data) {
                        if (last_positions.find(tracker_id) != last_positions.end()) {
                            auto& last_pos = last_positions[tracker_id];
                            float dx = data.position[0] - last_pos[0];
                            float dy = data.position[1] - last_pos[1];
                            float dz = data.position[2] - last_pos[2];
                            float movement = std::sqrt(dx*dx + dy*dy + dz*dz);
                            tracker_movement[tracker_id] = movement;
                        }
                        // Update last position
                        last_positions[tracker_id] = data.position;
                    }
                }
                
                // Find tracker with most movement
                float max_movement = 0.0f;
                std::string most_active_tracker;
                
                for (const auto& [tracker_id, movement] : tracker_movement) {
                    if (movement > max_movement) {
                        max_movement = movement;
                        most_active_tracker = tracker_id;
                    }
                }
                
                // Time-based tracker detection measuring once per second
                static auto last_movement_check = std::chrono::steady_clock::now();
                static auto last_display_time = std::chrono::steady_clock::now();
                auto now = std::chrono::steady_clock::now();
                
                auto time_since_last_check = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_movement_check).count();
                auto time_since_last_update = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_display_time).count();
                
                // Maintain movement tracking based on time
                static std::unordered_map<std::string, int> significant_movement_seconds;
                static std::unordered_map<std::string, std::array<float, 3>> previous_positions;
                static std::unordered_map<std::string, float> total_movement;
                
                // Capture reference positions if we haven't yet
                if (previous_positions.empty()) {
                    std::lock_guard<std::mutex> lock(tracker_mutex);
                    for (const auto& [tracker_id, data] : tracker_data) {
                        previous_positions[tracker_id] = data.position;
                    }
                    last_movement_check = now;
                }
                
                // Log positions to file (every frame) - Using reliable file writer
                {
                    std::lock_guard<std::mutex> lock(tracker_mutex);
                    
                    // Log each tracker position
                    if (position_log.is_open()) {
                        for (const auto& [tracker_id, data] : tracker_data) {
                            // Use our reliable file writer
                            bool success = FileWriter::logPosition(
                                position_log, 
                                tracker_id,
                                data.position[0],
                                data.position[1],
                                data.position[2]
                            );
                            
                            // Every 20 position logs, show a status message
                            static int log_counter = 0;
                            if (++log_counter % 20 == 0) {
                                if (success) {
                                    std::cout << "✓ Position logged for tracker: " << tracker_id 
                                              << " at [" << data.position[0] << ", " 
                                              << data.position[1] << ", " 
                                              << data.position[2] << "]" << std::endl;
                                } else {
                                    std::cerr << "✗ Failed to log position for tracker: " 
                                              << tracker_id << std::endl;
                                }
                            }
                        }
                    } else {
                        // If file is not open, try to reopen it
                        position_log = FileWriter::createPositionLog(drone.ip);
                        
                        if (position_log.is_open()) {
                            std::cout << "Reopened position log file after it was unexpectedly closed" << std::endl;
                        } else {
                            std::cerr << "ERROR: Failed to reopen position log file!" << std::endl;
                        }
                    }
                }
                
                // Check movement once per second
                if (time_since_last_check >= 1000) { // Check once per second
                    std::lock_guard<std::mutex> lock(tracker_mutex);
                    
                    // Check all trackers for movement
                    for (const auto& [tracker_id, data] : tracker_data) {
                        if (previous_positions.find(tracker_id) != previous_positions.end()) {
                            auto& prev_pos = previous_positions[tracker_id];
                            
                            // Calculate total displacement over the last second
                            float dx = data.position[0] - prev_pos[0];
                            float dy = data.position[1] - prev_pos[1];
                            float dz = data.position[2] - prev_pos[2];
                            float displacement = std::sqrt(dx*dx + dy*dy + dz*dz);
                            
                            // Update total movement
                            total_movement[tracker_id] += displacement;
                            
                            // Check if movement exceeds threshold (lowered from 0.5cm to 0.2cm for better sensitivity)
                            if (displacement > 0.002f) {
                                significant_movement_seconds[tracker_id]++;
                                // Debug print for significant movement
                                std::cout << "Motion detected for tracker: " << tracker_id 
                                          << " displacement=" << displacement << "m" << std::endl;
                            }
                            
                            // Update previous position for next check
                            previous_positions[tracker_id] = data.position;
                        }
                    }
                    
                    // Reset timer for next check
                    last_movement_check = now;
                }
                
                // Find tracker with most significant seconds of movement
                int max_seconds = 0;
                std::string consistently_moving_tracker;
                
                for (const auto& [tracker_id, seconds] : significant_movement_seconds) {
                    if (seconds > max_seconds && seconds >= 3) { // Require 3 seconds of significant movement
                        max_seconds = seconds;
                        consistently_moving_tracker = tracker_id;
                    }
                }
                
                // Update display less frequently to prevent flickering
                // Only update when there's significant information to show or after a longer interval
                if (!consistently_moving_tracker.empty() || time_since_last_update > 1000) {
                    last_display_time = now;
                    
                    // Use a more stable approach than system("clear")
                    std::cout << "\033[2J\033[1;1H";  // ANSI escape sequence to clear screen and move cursor to top
                    std::cout.flush();
                    
                    std::cout << "Mapping drone " << drone.ip << " (MAC: " << drone.mac << ")\n\n";
                    std::cout << "--- Flight Controls ---\n";
                    std::cout << "  'w/a/s/d' - forward/left/backward/right\n";
                    std::cout << "  'up/down arrows' - up/down\n";
                    std::cout << "  'left/right arrows' - rotate left/right\n";
                    std::cout << "  'space' - hover in place\n";
                    std::cout << "  'l' - land and confirm tracking\n";
                    std::cout << "  'q' - land and skip this drone\n\n";
                    
                    // Show consistently moving tracker with more prominent display
                    if (!consistently_moving_tracker.empty()) {
                        std::cout << "▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓\n";
                        std::cout << "  DETECTED DRONE: " << consistently_moving_tracker << "\n";
                        std::cout << "  Movement detected for: " << significant_movement_seconds[consistently_moving_tracker] << " seconds\n";
                        std::cout << "  Total displacement: " << total_movement[consistently_moving_tracker] << " meters\n";
                        std::cout << "▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓\n\n";
                        
                        // Remember the consistently moving tracker
                        moved_tracker = consistently_moving_tracker;
                    } else {
                        // List all trackers with their movement counts
                        std::cout << "Tracker movement status:\n";
                        for (const auto& [tracker_id, seconds] : significant_movement_seconds) {
                            std::cout << "  " << tracker_id << ": " 
                                      << seconds << " seconds of movement, "
                                      << total_movement[tracker_id] << " meters total\n";
                        }
                        std::cout << "\nWaiting for consistent movement (need 3+ seconds)...\n";
                    }
                }
                
                // Small delay to prevent CPU overuse
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
            }
            
            // Always restore original terminal settings when exiting flight control
            tcsetattr(STDIN_FILENO, TCSAFLUSH, &orig_flight_tty);
            fcntl(STDIN_FILENO, F_SETFL, 0); // Remove non-blocking flag
            
            // Print clear message that we're exiting flight control mode
            std::cout << "\n--- Exiting flight control mode ---\n" << std::flush;
            
            // Land the drone using direct socket communication
            std::cout << "Landing drone...\n";
            
            // Create a direct socket for landing
            int land_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
            if (land_sock < 0) {
                std::cerr << "Failed to create socket for landing: " << strerror(errno) << std::endl;
            } else {
                // Set up address
                struct sockaddr_in addr{};
                addr.sin_family = AF_INET;
                addr.sin_addr.s_addr = inet_addr(drone.ip.c_str());
                addr.sin_port = htons(8889);
                
                // Send land command directly
                std::string land_cmd = "land";
                sendto(land_sock, land_cmd.c_str(), land_cmd.length(), 0, 
                       (struct sockaddr*)&addr, sizeof(addr));
                
                close(land_sock);
            }
            
            // Also try the normal command in case direct fails
            sendCommandBlocking(drone.ip, "land");
            
            // Wait for landing
            std::cout << "Landing command sent, waiting for drone to land...\n";
            std::this_thread::sleep_for(std::chrono::seconds(3));
            
            drone.is_flying = false;
            
            // Restore terminal settings to normal mode before returning to menu
            struct termios tty;
            tcgetattr(STDIN_FILENO, &tty);
            tty.c_lflag |= (ICANON | ECHO); // Re-enable canonical mode and echo
            tcsetattr(STDIN_FILENO, TCSAFLUSH, &tty);
            fcntl(STDIN_FILENO, F_SETFL, 0); // Remove non-blocking flag
            
            // Close position log file
            if (position_log.is_open()) {
                position_log.close();
                std::cout << "Position log file closed. Logged data saved to " << log_filename << std::endl;
            }
            
            // If user confirmed a tracker by pressing 'l'
            if (confirm_tracker && !moved_tracker.empty()) {
                std::cout << "\nMapping confirmed: drone " << drone.ip << " -> tracker " << moved_tracker << "\n";
                
                // Set the tracker_id directly and verify it was set
                drone.tracker_id = moved_tracker;
                
                std::cout << "Setting tracker ID for drone " << drone.ip << " (MAC: " << drone.mac 
                          << ") to: " << moved_tracker << std::endl;
                
                // Use our reliable file writer to save the mapping
                bool success = FileWriter::saveTrackerMapping(drone.mac, moved_tracker);
                
                if (success) {
                    std::cout << "✓ Successfully saved drone-to-tracker mapping" << std::endl;
                    
                    // Also save with the original method as backup
                    FileHandler::saveMapping(drones);
                } else {
                    std::cerr << "WARNING: Failed to save mapping with direct file writer" << std::endl;
                    std::cout << "Attempting to save with original method..." << std::endl;
                    FileHandler::saveMapping(drones);
                }
                
                // Verify the mapping was applied to the drone object
                if (drone.tracker_id == moved_tracker) {
                    std::cout << "✓ Tracker ID correctly set in drone object" << std::endl;
                } else {
                    std::cerr << "ERROR: Tracker ID not correctly set in drone object!" << std::endl;
                }
            } else {
                std::cout << "\nMapping skipped for drone " << drone.ip << "\n";
            }
            
            std::this_thread::sleep_for(std::chrono::seconds(1));
        
        // Kill the visualizer process
        if (viz_pid > 0) {
            kill(viz_pid, SIGTERM);
            // Wait for it to exit
            int status;
            waitpid(viz_pid, &status, 0);
        }
        
        // Save mapping again as a final check
        if (!drone.tracker_id.empty()) {
            std::cout << "Final verification - Saving drone " << drone.mac << " mapping to tracker " << drone.tracker_id << std::endl;
            FileHandler::saveMapping(drones);
            std::cout << "\nDrone-to-tracker mapping saved to: " << FileHandler::getMappingFilePath() << "\n";
        } else {
            std::cout << "\nNo mapping was created for this drone.\n";
        }
        
        // Ensure terminal is properly reset before returning to menu
        // Note: We've already reset terminal above in the drone landing code, but let's do it again for safety
        {
            // Use a new scope to avoid variable conflicts
            struct termios menu_tty;
            tcgetattr(STDIN_FILENO, &menu_tty);
            menu_tty.c_lflag |= (ICANON | ECHO); // Re-enable canonical mode and echo
            tcsetattr(STDIN_FILENO, TCSAFLUSH, &menu_tty);
            fcntl(STDIN_FILENO, F_SETFL, 0); // Remove non-blocking flag
        }
        
        // Force a small delay to ensure terminal reset takes effect
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        
        // Clear screen and reset cursor 
        std::cout << "\033[2J\033[1;1H" << std::flush;
        
        std::cout << "\nPress Enter to return to menu...";
        std::cout.flush();
        
        // Discard any remaining input and reset cin
        std::cin.clear();
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        
        // Wait for Enter key
        std::cin.get();
        
        // Another small pause before returning
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        
        return true;
    }

    // Set reference frame for orientation calibration
    bool calibrateReferenceFrame() {
        if (!has_vrpn_data.load()) {
            std::cout << "ERROR: No VRPN tracking data available. Cannot calibrate reference frame.\n"
                     << "Make sure the OptiTrack system is running and the VRPN server is connected.\n";
            std::cout << "\nPress any key to return to menu...";
            std::cin.get();
            return false;
        }
        
        // Count connected drones
        int connected_count = 0;
        for (const auto& drone : drones) {
            if (drone.is_connected) connected_count++;
        }
        
        if (connected_count == 0) {
            std::cout << "ERROR: No drones are responsive. Cannot calibrate reference frame.\n"
                     << "Make sure drones are powered on and try scanning again.\n";
            std::cout << "\nPress any key to return to menu...";
            std::cin.get();
            return false;
        }
        
        // Check if all connected drones have trackers assigned
        for (const auto& drone : drones) {
            if (drone.is_connected && drone.tracker_id.empty()) {
                std::cout << "ERROR: Not all connected drones have tracker IDs assigned.\n"
                         << "Run the 'Map Drones to OptiTrack' option first.\n";
                std::cout << "\nPress any key to return to menu...";
                std::cin.get();
                return false;
            }
        }
        
        std::cout << "Calibrating reference frame...\n\n";
        std::cout << "This will establish the orientation reference frame.\n";
        std::cout << "Place a drone on the ground in the desired forward orientation.\n";
        
        // Ask user to select a drone
        std::cout << "Select a connected drone to use as reference:\n";
        std::vector<size_t> connected_indices;
        for (size_t i = 0; i < drones.size(); i++) {
            if (drones[i].is_connected) {
                connected_indices.push_back(i);
                std::cout << connected_indices.size() << ": " << drones[i].ip 
                          << " (" << drones[i].mac << ") - CONNECTED\n";
            }
        }
        
        size_t selection = 0;
        std::cout << "Enter drone number (1-" << connected_indices.size() << "): ";
        std::cin >> selection;
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');  // Clear input buffer
        
        if (selection < 1 || selection > connected_indices.size()) {
            std::cout << "Invalid choice.\n";
            std::cout << "\nPress any key to return to menu...";
            std::cin.get();
            return false;
        }
        
        auto& reference_drone = drones[connected_indices[selection - 1]];
        
        // Flash LED to identify the drone
        sendCommandBlocking(reference_drone.ip, "led 255 0 0");
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        sendCommandBlocking(reference_drone.ip, "led 0 0 0");
        
        std::cout << "\nPlace drone " << reference_drone.ip << " facing the desired forward direction.\n";
        std::cout << "Press Enter when ready...";
        std::cin.get();
        
        // Capture orientation from tracker
        {
            std::lock_guard<std::mutex> lock(tracker_mutex);
            if (tracker_data.find(reference_drone.tracker_id) != tracker_data.end()) {
                auto& data = tracker_data[reference_drone.tracker_id];
                reference_orientation = data.rotation;
                
                std::cout << "Captured reference orientation:\n";
                std::cout << "Quaternion: [" << std::fixed << std::setprecision(4)
                          << (*reference_orientation)[0] << ", "
                          << (*reference_orientation)[1] << ", "
                          << (*reference_orientation)[2] << ", "
                          << (*reference_orientation)[3] << "]\n";
                
                // Save reference frame
                FileHandler::saveReferenceFrame(*reference_orientation);
                
                std::cout << "\nReference frame calibrated and saved.\n";
            } else {
                std::cout << "ERROR: Could not get tracker data for " << reference_drone.tracker_id << ".\n";
                reference_orientation = std::nullopt;
                std::cout << "\nCalibration failed!\n";
                std::cout << "\nPress any key to return to menu...";
                std::cin.get();
                return false;
            }
        }
        
        std::cout << "\nPress any key to return to menu...";
        std::cin.get();
        return true;
    }

    // Run drift calibration
    bool calibrateDrift() {
        // Ensure we have drones
        if (drones.empty()) {
            std::cout << "No drones found! Make sure they are connected to the network.\n";
            std::cout << "\nPress any key to return to menu...";
            std::cin.get();
            return false;
        }
        
        // Count connected drones
        int connected_count = 0;
        for (const auto& drone : drones) {
            if (drone.is_connected) connected_count++;
        }
        
        if (connected_count == 0) {
            std::cout << "ERROR: No drones are responsive. Cannot calibrate drift.\n"
                     << "Make sure drones are powered on and try scanning again.\n";
            std::cout << "\nPress any key to return to menu...";
            std::cin.get();
            return false;
        }
        
        KeyboardInput keyboard;
        
        // Find the first connected drone
        current_drone = 0;
        for (size_t i = 0; i < drones.size(); i++) {
            if (drones[i].is_connected) {
                current_drone = i;
                break;
            }
        }
        
        // Prevent hanging - set up a timer to reset if needed
        auto last_activity = std::chrono::steady_clock::now();
        
        std::cout << "TIPS FOR BETTER DRIFT CALIBRATION:\n";
        std::cout << "1. Values now range from -100 to 100 instead of -5 to 5\n";
        std::cout << "2. Try larger values like 30-50 if needed\n";
        std::cout << "3. Press CTRL+C to force quit at any time if the program freezes\n";
        std::cout << "4. Space lands all drones immediately (emergency stop)\n\n";
        std::cout << "Press any key to begin...";
        std::cin.get();
        
        while (running.load() && current_drone < drones.size()) {
            displayStatus();
            
            // Check for emergency stop
            int key = keyboard.getKey();
            
            // Check if program has been active recently
            auto now = std::chrono::steady_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::seconds>(now - last_activity).count();
            
            // If we haven't processed input in 5 seconds, check if there's an escape key waiting
            if (duration > 5) {
                // Force read from stdin to see if there's an escape key waiting
                char c;
                if (read(STDIN_FILENO, &c, 1) > 0 && c == 27) {
                    std::cout << "Force exit detected!" << std::endl;
                    emergencyLandAll();
                    return false;
                }
                last_activity = now;
            }
            
            if (key == ' ') {
                // If space is hit, force land ALL drones
                emergencyLandAll();
                std::cout << "\nAll drones landed.\n";
                continue;
            }
            
            // If we got valid input, update activity time
            if (key != -1) {
                last_activity = now;
            }
            
            if (current_drone >= drones.size()) {
                std::cout << "No more drones to calibrate" << std::endl;
                break;
            }

            auto& drone = drones[current_drone];

            if (key != -1) {
                switch (key) {
                    case 'w': // Pitch forward
                        drone.pitch_bias += coarse_adjust;
                        applyBias(drone);
                        break;
                    case 'x': // Pitch backward
                        drone.pitch_bias -= coarse_adjust;
                        applyBias(drone);
                        break;
                    case 'a': // Roll left
                        drone.roll_bias -= coarse_adjust;
                        applyBias(drone);
                        break;
                    case 'd': // Roll right
                        drone.roll_bias += coarse_adjust;
                        applyBias(drone);
                        break;
                    case 'q': // Yaw left
                        drone.yaw_bias -= coarse_adjust;
                        applyBias(drone);
                        break;
                    case 'e': // Yaw right
                        drone.yaw_bias += coarse_adjust;
                        applyBias(drone);
                        break;
                    case 'i': // Fine pitch forward
                        drone.pitch_bias += fine_adjust;
                        applyBias(drone);
                        break;
                    case 'k': // Fine pitch backward
                        drone.pitch_bias -= fine_adjust;
                        applyBias(drone);
                        break;
                    case 'j': // Fine roll left
                        drone.roll_bias -= fine_adjust;
                        applyBias(drone);
                        break;
                    case 'l': // Fine roll right
                        drone.roll_bias += fine_adjust;
                        applyBias(drone);
                        break;
                    case 'u': // Fine yaw left
                        drone.yaw_bias -= fine_adjust;
                        applyBias(drone);
                        break;
                    case 'o': // Fine yaw right
                        drone.yaw_bias += fine_adjust;
                        applyBias(drone);
                        break;
                    case 't': // Takeoff
                        if (!drone.is_flying) {
                            std::string response = sendCommandBlocking(drone.ip, "takeoff");
                            if (response == "ok") {
                                drone.is_flying = true;
                                applyBias(drone);  // Apply bias immediately after takeoff
                            } else {
                                std::cout << "\nTakeoff failed: " << response << std::endl;
                            }
                        }
                        break;
                    case ' ': // Land current drone
                        if (drone.is_flying) {
                            sendCommandBlocking(drone.ip, "land");
                            drone.is_flying = false;
                        }
                        break;
                    case 'n': // Next drone
                        {
                            if (drone.is_flying) {
                                sendCommandBlocking(drone.ip, "land");
                                drone.is_flying = false;
                            }
                            // Find next connected drone
                            size_t next_drone = current_drone;
                            bool found_next = false;
                            
                            for (size_t i = current_drone + 1; i < drones.size(); i++) {
                                if (drones[i].is_connected) {
                                    next_drone = i;
                                    found_next = true;
                                    break;
                                }
                            }
                            
                            if (!found_next) {
                                // Loop back to first drone if needed
                                for (size_t i = 0; i < current_drone; i++) {
                                    if (drones[i].is_connected) {
                                        next_drone = i;
                                        found_next = true;
                                        break;
                                    }
                                }
                            }
                            
                            if (found_next && next_drone != current_drone) {
                                current_drone = next_drone;
                                std::cout << "Moving to next drone: " << drones[current_drone].ip << std::endl;
                            } else {
                                std::cout << "No other connected drones available." << std::endl;
                            }
                            break;
                        }
                    case 's': // Save and quit
                        std::cout << "Saving calibration..." << std::endl;
                        if (drone.is_flying) {
                            sendCommandBlocking(drone.ip, "land");
                            drone.is_flying = false;
                        }
                        FileHandler::saveCalibration(drones);
                        running.store(false);
                        break;
                    case 'r': // Reset values
                        if (drone.is_flying) {
                            sendCommandBlocking(drone.ip, "land");
                            drone.is_flying = false;
                        }
                        drone.pitch_bias = 0;
                        drone.roll_bias = 0;
                        drone.yaw_bias = 0;
                        break;
                    case 27: // Escape key (quit without saving)
                        if (drone.is_flying) {
                            sendCommandBlocking(drone.ip, "land");
                            drone.is_flying = false;
                        }
                        running.store(false);
                        break;
                }
            }

            if (drone.is_flying) {
                applyBias(drone);
            }   
            
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        
        // Ensure all drones are landed before exit
        for (auto& drone : drones) {
            if (drone.is_flying) {
                sendCommandBlocking(drone.ip, "land");
                drone.is_flying = false;
            }
        }
        
        // Reset terminal to canonical mode
        struct termios tty;
        tcgetattr(STDIN_FILENO, &tty);
        tty.c_lflag |= (ICANON | ECHO); // Re-enable canonical mode and echo
        tcsetattr(STDIN_FILENO, TCSAFLUSH, &tty);
        fcntl(STDIN_FILENO, F_SETFL, 0); // Remove non-blocking flag
        
        // Clear screen and reset cursor
        std::cout << "\033[2J\033[1;1H" << std::flush;
        
        return true;
    }

    // Initialize all drones by testing connection
    void initializeDrones() {
        std::cout << "Initializing drones..." << std::endl;
        int connected_count = 0;
        int online_count = 0;
        
        // Initialize sockets and verify connectivity only for online drones
        for (auto& drone : drones) {
            if (!drone.is_online) {
                std::cout << "Skipping offline drone with MAC: " << drone.mac << " (last seen: " << drone.last_seen << ")" << std::endl;
                continue;
            }
            
            online_count++;
            std::cout << "Connecting to " << drone.ip << "..." << std::endl;
            drone.is_connected = false; // Reset connection status
            
            // Try multiple times to initialize socket
            bool socket_initialized = false;
            for (int attempt = 1; attempt <= 3; attempt++) {
                if (initSocket(drone)) {
                    socket_initialized = true;
                    break;
                }
                std::cerr << "Failed to initialize socket for " << drone.ip 
                         << " (attempt " << attempt << "/3)" << std::endl;
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
            }
            
            if (!socket_initialized) {
                std::cerr << "Could not initialize socket for " << drone.ip 
                         << " after multiple attempts. Skipping this drone." << std::endl;
                continue;
            }

            // If drone was found in scan, assume it's ready to use
            drone.is_connected = true;
            connected_count++;
            
            std::cout << "Successfully initialized drone at " << drone.ip << " (MAC: " << drone.mac << ")" << std::endl;
        }
        
        std::cout << "\nInitialization complete: " << connected_count << " of " 
                  << online_count << " online drones are initialized." << std::endl;
        
        if (drones.size() > online_count) {
            std::cout << "Also found " << (drones.size() - online_count) 
                      << " offline drones with saved calibration data." << std::endl;
        }
                  
        if (connected_count == 0 && !drones.empty()) {
            std::cout << "\nWARNING: No drones could be initialized! Please check:" << std::endl;
            std::cout << "1. Are the drones powered on?" << std::endl;
            std::cout << "2. Are you connected to the same WiFi network?" << std::endl;
            std::cout << "3. Consider rescanning the network or resetting the drones" << std::endl;
        }
    }

    // Show the main menu
    void showMenu() {
        while (true) {
            system("clear");
            std::cout << "Drone Management System\n";
            std::cout << "======================\n\n";
            std::cout << "1. Load Drones from Configuration\n";
            std::cout << "2. Scan Network for Drones\n";
            std::cout << "3. Map Drones to OptiTrack\n";
            std::cout << "4. Calibrate Reference Frame\n";
            std::cout << "5. Drift Calibration\n";
            std::cout << "6. Exit\n\n";
            
            // Count actually connected drones
            int connected_count = 0;
            for (const auto& drone : drones) {
                if (drone.is_connected) connected_count++;
            }
            
            // Count online drones
            int online_count = 0;
            for (const auto& drone : drones) {
                if (drone.is_online) online_count++;
            }
            
            std::cout << "Drones: " << connected_count << " connected, " 
                      << online_count - connected_count << " unresponsive online, "
                      << drones.size() - online_count << " saved offline, "
                      << drones.size() << " total\n";
            std::cout << "OptiTrack status: " << (has_vrpn_data.load() ? "Connected" : "Not connected") << "\n\n";
            
            std::cout << "Enter choice (1-6): ";
            
            int choice;
            std::cin >> choice;
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');  // Clear input buffer
            
            switch (choice) {
                case 1:
                    // Load drones from existing configuration
                    drones = FileHandler::findDrones(false);
                    if (drones.empty()) {
                        std::cout << "No drones found in configuration! Scan for drones first.\n";
                    } else {
                        // Load existing calibration data
                        FileHandler::loadCalibration(drones);
                        
                        // Initialize connections
                        initializeDrones();
                    }
                    std::cout << "\nPress any key to continue...";
                    std::cin.get();
                    break;
                
                case 2:
                    // Actively scan network for drones
                    std::cout << "Scanning network for DJI drones...\n";
                    drones = FileHandler::findDrones(true);
                    if (drones.empty()) {
                        std::cout << "No drones found! Make sure they are connected to the network.\n";
                    } else {
                        // Load existing calibration data
                        FileHandler::loadCalibration(drones);
                        
                        // Initialize connections
                        initializeDrones();
                    }
                    std::cout << "\nPress any key to continue...";
                    std::cin.get();
                    break;
                
                case 3:
                    if (drones.empty()) {
                        std::cout << "No drones found! Load or scan for drones first.\n";
                        std::cout << "\nPress any key to continue...";
                        std::cin.get();
                    } else {
                        mapSingleDrone();
                    }
                    break;
                
                case 4:
                    if (drones.empty()) {
                        std::cout << "No drones found! Load or scan for drones first.\n";
                        std::cout << "\nPress any key to continue...";
                        std::cin.get();
                    } else {
                        calibrateReferenceFrame();
                    }
                    break;
                
                case 5:
                    if (drones.empty()) {
                        std::cout << "No drones found! Load or scan for drones first.\n";
                        std::cout << "\nPress any key to continue...";
                        std::cin.get();
                    } else {
                        running.store(true);
                        calibrateDrift();
                        running.store(false);
                    }
                    break;
                
                case 6:
                    // Verify all drones are landed
                    for (auto& drone : drones) {
                        if (drone.is_flying) {
                            sendCommandBlocking(drone.ip, "land");
                            drone.is_flying = false;
                        }
                    }
                    running.store(false);
                    return;
                
                default:
                    std::cout << "Invalid choice. Please try again.\n";
                    std::cout << "\nPress any key to continue...";
                    std::cin.get();
                    break;
            }
        }
    }

public:
    DroneManager() {
        std::cout << "Starting Drone Manager..." << std::endl;
        
        // Use larger adjustment values for Tello drones
        coarse_adjust = 10.0f;  // Use larger values 
        fine_adjust = 1.0f;     // Fine adjustment = 1 unit
        
        // Load drone configurations
        drones = FileHandler::findDrones();
        if (drones.empty()) {
            std::cout << "No drones found! Make sure they are connected to the network." << std::endl;
            std::cout << "Run python/get_devices.py first if needed." << std::endl;
        } else {
            // Load calibration data
            FileHandler::loadCalibration(drones);
            
            // Try to load reference orientation
            reference_orientation = FileHandler::loadReferenceFrame();
            if (reference_orientation) {
                std::cout << "Loaded reference orientation" << std::endl;
            }
            
            // Initialize connections
            initializeDrones();
        }
        
        // Start command processor thread
        command_thread = std::thread(&DroneManager::commandProcessor, this);
        
        // Start flight watchdog thread
        watchdog_thread = std::thread(&DroneManager::flightWatchdog, this);
        
        // Try to connect to VRPN server
        if (connectVrpn()) {
            std::cout << "Connected to VRPN server for tracking data" << std::endl;
        } else {
            std::cout << "Warning: Failed to connect to VRPN server. Continuing without tracking data." << std::endl;
        }
    }

    void run() {
        // Present main menu
        showMenu();
    }

    ~DroneManager() {
        std::cout << "Shutting down Drone Manager..." << std::endl;
        
        // Set running to false to signal threads to stop
        running.store(false);
        
        // Stop command queue and join threads
        command_queue.stopQueue();
        
        if (command_thread.joinable()) {
            command_thread.join();
        }
        
        if (vrpn_thread.joinable()) {
            vrpn_thread.join();
        }
        
        if (watchdog_thread.joinable()) {
            watchdog_thread.join();
        }
        
        // Ensure all drones are landed
        for (auto& drone : drones) {
            if (drone.is_flying) {
                std::cout << "Landing drone " << drone.ip << "..." << std::endl;
                
                // Direct socket send since threads are stopped
                int sock = drone_sockets[drone.ip];
                if (sock >= 0) {
                    std::string cmd = "land";
                    sendto(sock, cmd.c_str(), cmd.length(), 0,
                         (struct sockaddr *)&drone_addresses[drone.ip], 
                         sizeof(drone_addresses[drone.ip]));
                }
                
                drone.is_flying = false;
            }
        }
        
        // Close all sockets
        for (const auto& [ip, sock] : drone_sockets) {
            if (sock >= 0) {
                close(sock);
            }
        }
    }
};

int main() {
    // Setup signal handler to restore terminal on Ctrl+C
    struct sigaction sa;
    sa.sa_handler = [](int sig) {
        // Reset terminal to canonical mode on signal
        struct termios tty;
        tcgetattr(STDIN_FILENO, &tty);
        tty.c_lflag |= (ICANON | ECHO); // Re-enable canonical mode and echo
        tcsetattr(STDIN_FILENO, TCSAFLUSH, &tty);
        fcntl(STDIN_FILENO, F_SETFL, 0); // Remove non-blocking flag
        
        // Clear screen and reset cursor
        std::cout << "\033[2J\033[1;1H" << std::flush;
        
        // Exit with signal code
        exit(sig);
    };
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;
    sigaction(SIGINT, &sa, NULL);  // Handle Ctrl+C
    sigaction(SIGTERM, &sa, NULL); // Handle termination signal
    
    // Save original terminal state for restoration at end
    struct termios orig_tty;
    tcgetattr(STDIN_FILENO, &orig_tty);
    
    try {
        DroneManager manager;
        manager.run();
    } catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
    } catch (...) {
        std::cerr << "Unknown exception occurred" << std::endl;
    }
    
    // Always restore terminal settings, no matter how we exit
    tcsetattr(STDIN_FILENO, TCSAFLUSH, &orig_tty);
    return 0;
}