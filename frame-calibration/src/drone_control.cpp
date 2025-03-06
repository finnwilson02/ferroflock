#include <fstream>
#include <string>
#include <chrono>
#include <iomanip>
#include <thread>
#include <iostream>
#include <vector>
#include <sstream>
#include <algorithm>   // for std::min, std::max
#include <random>      // for random number generation
#include <filesystem>  // for directory creation
#include <csignal>     // for signal handling
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <regex>       // for simple JSON parsing
#include <cstdlib>     // for atexit
#include <cctype>      // for std::toupper
#include <map>         // for std::map
#include <cstring>
#include "../include/optitrack_viz.h"

// Logging functions
#define LOG_INFO(msg) std::cout << "[DRONE_CONTROL][INFO] " << msg << std::endl
#define LOG_ERROR(msg) std::cerr << "[DRONE_CONTROL][ERROR] " << msg << std::endl
#define LOG_DEBUG(msg) std::cout << "[DRONE_CONTROL][DEBUG] " << msg << std::endl
#define LOG_WARNING(msg) std::cout << "[DRONE_CONTROL][WARNING] " << msg << std::endl

// Forward declarations
void init_optitrack_viz();
void update_optitrack_viz(double x, double y, double yaw);
void stop_visualization();

// Extern declarations for optitrack_viz.cpp functions
extern double get_optitrack_x_for_name(const std::string& name);
extern double get_optitrack_y_for_name(const std::string& name);
extern double get_optitrack_yaw_for_name(const std::string& name);

// Simple drone configuration structure
struct DroneConfig {
    std::string id;
    std::string ip;
    std::string optitrack_name; // Added OptiTrack rigid body name
};

// Forward declare DroneControl class
class DroneControl;

// Global vector to store all drones
std::vector<DroneConfig> allDrones;

// Simple JSON parser for drone configuration
std::vector<DroneConfig> loadDronesFromJSON(const std::string& filename) {
    std::vector<DroneConfig> drones;
    std::ifstream json_file(filename);
    
    if (!json_file.is_open()) {
        std::cerr << "Failed to open " << filename << std::endl;
        return drones;
    }
    
    // Create hardcoded mappings based on the IP addresses that match the requirements
    // Bird5 (107), Bird3 (106), Bird4 (104), Bird1 (108)
    std::map<std::string, std::string> ipToOptitrackMap = {
        {"192.168.1.107", "Bird5"},
        {"192.168.1.106", "Bird3"},
        {"192.168.1.104", "Bird4"},
        {"192.168.1.108", "Bird1"}
    };
    
    // Read the entire file into a string
    std::stringstream buffer;
    buffer << json_file.rdbuf();
    std::string content = buffer.str();
    
    // Just parse IP addresses from the file
    std::regex ip_pattern("\"ip\"\\s*:\\s*\"([^\"]+)\"");
    
    auto ips_begin = std::sregex_iterator(content.begin(), content.end(), ip_pattern);
    auto ips_end = std::sregex_iterator();
    
    int drone_count = 0;
    for (std::sregex_iterator i = ips_begin; i != ips_end; ++i) {
        std::smatch match = *i;
        if (match.size() > 1) {
            std::string ip = match[1].str();
            DroneConfig drone;
            drone.ip = ip;
            drone.id = "drone" + std::to_string(++drone_count);
            
            // Assign OptiTrack name based on IP
            if (ipToOptitrackMap.find(ip) != ipToOptitrackMap.end()) {
                drone.optitrack_name = ipToOptitrackMap[ip];
            } else {
                drone.optitrack_name = "Bird" + std::to_string(drone_count);
            }
            
            std::cout << "Loaded drone: IP=" << drone.ip 
                      << ", ID=" << drone.id
                      << ", OptiTrack Name=" << drone.optitrack_name << std::endl;
            drones.push_back(drone);
        }
    }
    
    if (drones.empty()) {
        std::cerr << "No drones found in JSON" << std::endl;
    }
    
    return drones;
}

// Forward declarations
void rebootAllDrones();
void cleanup();
bool handleFlight(const std::string& drone_ip);

class DroneControl {
public:
    DroneControl(const std::string& drone_ip, const std::string& optitrack_name = "") 
        : drone_ip_(drone_ip), optitrack_name_(optitrack_name.empty() ? "Bird1" : optitrack_name) {
        LOG_DEBUG("Creating DroneControl for IP: " + drone_ip_ + ", OptiTrack: " + optitrack_name_);
        
        // Open the flight data CSV file
        LOG_DEBUG("Opening flight_data.csv for logging");
        file_.open("flight_data.csv", std::ios::app);
        if (!file_.is_open()) {
            LOG_ERROR("Failed to open flight_data.csv for writing");
        } else {
            LOG_DEBUG("Successfully opened flight_data.csv");
            
            // Write header if file is new
            if (file_.tellp() == 0) {
                LOG_DEBUG("Adding CSV header to flight_data.csv");
                file_ << "timestamp,commanded_yaw_deg,optitrack_x_m,optitrack_y_m,optitrack_yaw_deg,imu_yaw_deg\n";
                file_.flush();
            }
        }
        
        // Create command socket
        LOG_DEBUG("Creating UDP command socket");
        command_sock_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if (command_sock_ < 0) {
            LOG_ERROR("Failed to create command socket: " + std::string(strerror(errno)));
        } else {
            LOG_DEBUG("Command socket created successfully with descriptor: " + std::to_string(command_sock_));
        }
        
        // Start state receiver thread
        LOG_DEBUG("Starting state receiver thread");
        state_receiver_thread_ = std::thread(&DroneControl::state_receiver, this);
        LOG_DEBUG("DroneControl initialization complete");
    }

    ~DroneControl() {
        LOG_DEBUG("DroneControl destructor called for IP: " + drone_ip_);
        
        // Close the flight data file
        if (file_.is_open()) {
            LOG_DEBUG("Closing flight_data.csv");
            file_.close();
        }
        
        // Stop and join the state receiver thread
        LOG_DEBUG("Stopping state receiver thread");
        stop_state_receiver_ = true;
        if (state_receiver_thread_.joinable()) {
            LOG_DEBUG("Joining state receiver thread");
            state_receiver_thread_.join();
        }
        
        // Close the command socket
        if (command_sock_ >= 0) {
            LOG_DEBUG("Closing command socket: " + std::to_string(command_sock_));
            close(command_sock_);
        }
        
        // Clean up visualization
        LOG_DEBUG("Stopping OptiTrack visualization");
        stop_visualization();
        LOG_DEBUG("DroneControl cleanup complete");
    }

    void log(double timestamp, double cmd_yaw, double opt_x, double opt_y, double opt_yaw, double imu_yaw) {
        if (file_.is_open()) {
            // Log data to CSV
            file_ << std::fixed << std::setprecision(6) 
                  << timestamp << "," 
                  << cmd_yaw << ","
                  << opt_x << "," 
                  << opt_y << "," 
                  << opt_yaw << "," 
                  << imu_yaw << "\n";
            
            // Flush every 10 writes
            if (++write_count_ % 10 == 0) {
                LOG_DEBUG("Flushing flight_data.csv after " + std::to_string(write_count_) + " writes");
                file_.flush();
                
                // Check for file errors after flush
                if (file_.fail()) {
                    LOG_ERROR("Error detected after flushing flight_data.csv");
                    file_.clear(); // Clear error flags to continue writing
                }
            }
        } else {
            LOG_ERROR("Cannot log data - flight_data.csv is not open");
        }
    }
    
    // Reboot the drone
    void reboot() {
        std::cout << "Sending reboot command to drone..." << std::endl;
        send_command_to_drone("reboot");
        std::cout << "Drone reboot command sent." << std::endl;
    }
    
    // Check if flight data is available
    bool hasFlightData() const {
        // Implement this based on your requirements
        // For example, you might check if certain data was collected or if the flight completed
        return true; // Default to true for now
    }

    bool fly_and_log() {
        LOG_INFO("Starting fly_and_log for drone IP: " + drone_ip_ + ", OptiTrack: " + optitrack_name_);
        
        // Set up the flight timeline
        double timestamp = get_time();
        LOG_DEBUG("Start time: " + std::to_string(timestamp));
        
        double takeoff_end = timestamp + 2.0;         // 2s takeoff
        double takeoff_wait_end = takeoff_end + 5.0;  // 5s wait after takeoff
        double up_end = takeoff_wait_end + 2.0;       // 2s upward
        double up_wait_end = up_end + 1.0;            // 1s wait after up phase
        double forward_start = up_wait_end;           // Forward starts after wait
        double forward_end = forward_start + 5.0;     // 5s forward (increased from 2s)
        double forward_wait_end = forward_end + 1.0;  // 1s wait after forward
        double flight_end = forward_wait_end + 1.0;   // +1s after
        
        LOG_DEBUG("Flight timeline: takeoff_end=" + std::to_string(takeoff_end) + 
                 ", takeoff_wait_end=" + std::to_string(takeoff_wait_end) + 
                 ", up_end=" + std::to_string(up_end) + 
                 ", up_wait_end=" + std::to_string(up_wait_end) + 
                 ", forward_start=" + std::to_string(forward_start) + 
                 ", forward_end=" + std::to_string(forward_end) + 
                 ", forward_wait_end=" + std::to_string(forward_wait_end) + 
                 ", flight_end=" + std::to_string(flight_end));
        
        // Create the flight data directory if it doesn't exist
        std::string data_dir = "data";
        try {
            LOG_DEBUG("Creating data directory: " + data_dir);
            std::filesystem::create_directories(data_dir);
            LOG_DEBUG("Data directory created/verified successfully");
        } catch (const std::exception& e) {
            LOG_ERROR("Failed to create data directory: " + std::string(e.what()));
        }

        std::cout << "Starting flight sequence..." << std::endl;
        std::cout << "Press Ctrl+C to abort flight (emergency stop)" << std::endl;

        LOG_INFO("Setting up emergency signal handler");
        // Emergency handler setup - static to be accessible from signal handler
        static DroneControl* emergency_instance = this;
        
        // Set up a signal handler for emergency stop
        std::signal(SIGINT, [](int signal) {
            LOG_INFO("Received abort signal (SIGINT)");
            if (emergency_instance) {
                emergency_instance->emergency_stop();
            }
            LOG_INFO("Exiting after emergency stop");
            std::exit(signal);
        });
        
        LOG_INFO("Beginning flight execution");
        try {
            int log_count = 0; // Count how many data points we log
            
            // Main flight loop
            while ((timestamp = get_time()) < flight_end) {
                double cmd_yaw = 0.0; // Forward = 0°
                
                LOG_DEBUG("Getting OptiTrack data at t=" + std::to_string(timestamp));
                double opt_x = get_optitrack_x_for_name(optitrack_name_);
                double opt_y = get_optitrack_y_for_name(optitrack_name_);
                double opt_yaw = get_optitrack_yaw_for_name(optitrack_name_);
                
                // Check if we're getting valid OptiTrack data
                if (opt_x == 0.0 && opt_y == 0.0) {
                    LOG_WARNING("No OptiTrack data received for " + optitrack_name_ + " at t=" + std::to_string(timestamp));
                }
                
                double imu_yaw = get_imu_yaw();

                // Track the current phase to only print phase changes
                static int current_phase = -1;
                
                // Fly drone according to the current flight phase
                if (timestamp < takeoff_end) {
                    // Phase 0: Takeoff
                    if (current_phase != 0) {
                        current_phase = 0;
                        std::cout << "Taking off..." << std::endl;
                    }
                    send_command(0.0, 0.0, 1.0); // Takeoff (z up)
                } else if (timestamp < takeoff_wait_end) {
                    // Phase 1: Wait after takeoff
                    if (current_phase != 1) {
                        current_phase = 1;
                        std::cout << "Hovering after takeoff..." << std::endl;
                    }
                    send_command(0.0, 0.0, 0.0); // Hover after takeoff
                } else if (timestamp < up_end) {
                    // Phase 2: Up
                    if (current_phase != 2) {
                        current_phase = 2;
                        std::cout << "Moving upward..." << std::endl;
                    }
                    send_command(0.0, 0.0, 1.0); // Up
                } else if (timestamp < up_wait_end) {
                    // Phase 3: Wait after up
                    if (current_phase != 3) {
                        current_phase = 3;
                        std::cout << "Hovering after upward movement..." << std::endl;
                    }
                    send_command(0.0, 0.0, 0.0); // Hover after up phase
                } else if (timestamp < forward_end) {
                    // Phase 4: Forward
                    if (current_phase != 4) {
                        current_phase = 4;
                        std::cout << "Moving forward..." << std::endl;
                    }
                    send_command(1.0, 0.0, 0.0); // Forward (x forward)
                } else if (timestamp < forward_wait_end) {
                    // Phase 5: Wait after forward
                    if (current_phase != 5) {
                        current_phase = 5;
                        std::cout << "Hovering after forward movement..." << std::endl;
                    }
                    send_command(0.0, 0.0, 0.0); // Hover after forward phase
                } else {
                    // Phase 6: Extra logging
                    if (current_phase != 6) {
                        current_phase = 6;
                        std::cout << "Finishing flight sequence..." << std::endl;
                    }
                    // Keep hovering during the extra logging period
                    send_command(0.0, 0.0, 0.0);
                }

                // Log data during forward phase ±1s
                if (timestamp >= forward_start - 1.0 && timestamp <= forward_end + 1.0) {
                    LOG_DEBUG("Logging data point at t=" + std::to_string(timestamp) + 
                             ", forward_phase_time=" + std::to_string(timestamp - forward_start));
                    log(timestamp, cmd_yaw, opt_x, opt_y, opt_yaw, imu_yaw);
                    log_count++;
                    
                    // Log position data quality
                    if (opt_x == 0.0 && opt_y == 0.0) {
                        LOG_WARNING("Logging potentially invalid OptiTrack data (zeros)");
                    }
                }

                // Update visualization
                update_optitrack_viz(opt_x, opt_y, opt_yaw);
                
                // Maintain 10Hz control rate
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
            
            // Flight sequence completed, now land the drone
            LOG_INFO("Flight sequence completed, landing drone");
            land();
            
            // Make sure to flush any remaining log data
            if (file_.is_open()) {
                LOG_DEBUG("Final flush of flight_data.csv, wrote " + std::to_string(log_count) + " data points");
                file_.flush();
            }
            
            std::cout << "Flight complete." << std::endl;
            LOG_INFO("Flight successfully completed with " + std::to_string(log_count) + " logged data points");
            return true;
            
        } catch (const std::exception& e) {
            // If any exception occurs, attempt emergency landing
            LOG_ERROR("Exception during flight: " + std::string(e.what()));
            emergency_stop();
            
            // Try to flush any data we have
            if (file_.is_open()) {
                LOG_DEBUG("Emergency flush of flight_data.csv after error");
                file_.flush();
            }
            
            return false;
        }
    }

private:
    std::ofstream file_;
    int write_count_ = 0;
    
    int command_sock_ = -1;              // Persistent socket for sending commands
    std::string drone_ip_ = "192.168.10.1"; // Tello IP
    std::string optitrack_name_ = "Bird1"; // OptiTrack rigid body name
    int command_port_ = 8889;           // Tello command port
    double current_yaw_ = 0.0;          // Real-time IMU yaw from state messages
    std::thread state_receiver_thread_; // Thread for receiving state messages
    bool stop_state_receiver_ = false;  // Flag to stop the state receiver thread

    double get_time() { // Replace with your timing
        return std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count() / 1e6;
    }

    // NOTE: OptiTrack functions are now accessed directly from optitrack_viz.cpp
    
    void state_receiver() {
        int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if (sock < 0) {
            std::cerr << "Failed to create state socket" << std::endl;
            return;
        }

        struct sockaddr_in addr;
        addr.sin_family = AF_INET;
        addr.sin_port = htons(8890);
        addr.sin_addr.s_addr = INADDR_ANY;

        if (bind(sock, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
            std::cerr << "Failed to bind state socket" << std::endl;
            close(sock);
            return;
        }

        char buffer[1024];
        while (!stop_state_receiver_) {
            socklen_t len = sizeof(addr);
            int bytes_received = recvfrom(sock, buffer, sizeof(buffer) - 1, 0, 
                                         (struct sockaddr*)&addr, &len);
            if (bytes_received > 0) {
                buffer[bytes_received] = '\0';
                parse_state_message(std::string(buffer));
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        close(sock);
    }

    void parse_state_message(const std::string& message) {
        std::istringstream iss(message);
        std::string token;
        while (std::getline(iss, token, ';')) { // Tello state uses semicolons
            size_t colon_pos = token.find(':');
            if (colon_pos != std::string::npos) {
                std::string key = token.substr(0, colon_pos);
                std::string value_str = token.substr(colon_pos + 1);
                if (key == "yaw") {
                    try {
                        current_yaw_ = std::stod(value_str);
                    } catch (const std::exception& e) {
                        std::cerr << "Error parsing yaw: " << value_str << std::endl;
                    }
                }
            }
        }
    }
    
    // Real IMU data from drone state messages
    double get_imu_yaw() {
        return current_yaw_;
    }
    // Send command to the drone with additional features
    void send_command(double x, double y, double z) {
        static bool drone_initialized = false;
        static bool in_flight = false;
        static std::chrono::time_point<std::chrono::steady_clock> last_command_time;
        
        // Get current time
        auto now = std::chrono::steady_clock::now();
        
        // If this is the first command, initialize the drone
        if (!drone_initialized) {
            initialize_drone();
            drone_initialized = true;
            last_command_time = now;
            
            // Return early - don't send movement commands until initialized
            return;
        }
        
        // If z is positive and we're not in flight, we need to takeoff first
        if (z > 0.1 && !in_flight) {
            takeoff();
            in_flight = true;
            last_command_time = now;
            return;
        }
        
        // Scale inputs to half speed (-50 to 50 instead of -100 to 100)
        int x_scaled = static_cast<int>(x * 50);
        int y_scaled = static_cast<int>(y * 50);
        int z_scaled = static_cast<int>(z * 50);
        
        // Clamp values to valid range
        x_scaled = std::max(-50, std::min(50, x_scaled));
        y_scaled = std::max(-50, std::min(50, y_scaled));
        z_scaled = std::max(-50, std::min(50, z_scaled));
        
        // Use rc command for Tello drones: 
        // rc <left-right> <forward-backward> <up-down> <yaw>
        // We're not controlling yaw directly, so set it to a neutral value (0)
        std::ostringstream cmd_stream;
        cmd_stream << "rc " << y_scaled << " " << x_scaled << " " << z_scaled << " 0";
        std::string command = cmd_stream.str();
        
        // Send the command to the drone
        send_command_to_drone(command);
        
        // Update last command time
        last_command_time = now;
    }
    
    // Helper function to send commands to the drone
    void send_command_to_drone(const std::string& command) {
        if (command_sock_ < 0) {
            std::cerr << "Command socket not initialized" << std::endl;
            return;
        }
        struct sockaddr_in addr;
        addr.sin_family = AF_INET;
        addr.sin_port = htons(command_port_);
        addr.sin_addr.s_addr = inet_addr(drone_ip_.c_str());
        sendto(command_sock_, command.c_str(), command.length(), 0, 
               (struct sockaddr*)&addr, sizeof(addr));
        // No longer printing each command
    }
    
    // Initialize drone for flight
    void initialize_drone() {
        std::cout << "Initializing drone..." << std::endl;
        send_command_to_drone("command");
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    
    // Send takeoff command to the drone
    void takeoff() {
        send_command_to_drone("takeoff");
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    
    // Send land command to the drone
    void land() {
        std::cout << "Landing..." << std::endl;
        send_command_to_drone("land");
    }
    
    // Emergency stop for the drone
    void emergency_stop() {
        std::cout << "!!! EMERGENCY STOP ACTIVATED !!!" << std::endl;
        send_command_to_drone("emergency");
        std::cout << "Drone motors stopped" << std::endl;
    }
};

// Function to reboot all drones
void rebootAllDrones(bool wait_for_reboot = true, bool send_land_first = false) {
    std::cout << "Rebooting all drones...\n";
    
    if (allDrones.empty()) {
        std::cout << "No drones found to reboot.\n";
        return;
    }
    
    // Create sockets and send reboot commands to all drones
    std::vector<int> sockets;
    for (const auto& drone : allDrones) {
        if (send_land_first) {
            std::cout << "Sending land and reboot commands to drone " << drone.id 
                    << " (IP: " << drone.ip 
                    << ", OptiTrack: " << drone.optitrack_name << ")\n";
        } else {
            std::cout << "Sending reboot command to drone " << drone.id 
                    << " (IP: " << drone.ip 
                    << ", OptiTrack: " << drone.optitrack_name << ")\n";
        }
        
        // Create socket for this drone
        int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if (sock < 0) {
            std::cerr << "Failed to create socket for drone " << drone.id << std::endl;
            continue;
        }
        
        struct sockaddr_in addr;
        addr.sin_family = AF_INET;
        addr.sin_port = htons(8889); // Tello command port
        addr.sin_addr.s_addr = inet_addr(drone.ip.c_str());
        
        // Send land command first if requested (only when exiting)
        if (send_land_first) {
            std::string land_command = "land";
            sendto(sock, land_command.c_str(), land_command.length(), 0,
                  (struct sockaddr*)&addr, sizeof(addr));
            
            // Short delay to ensure land command is processed
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        
        // Send reboot command
        std::string reboot_command = "reboot";
        sendto(sock, reboot_command.c_str(), reboot_command.length(), 0, 
               (struct sockaddr*)&addr, sizeof(addr));
        
        if (wait_for_reboot) {
            // Save socket for cleanup later
            sockets.push_back(sock);
        } else {
            // Close socket immediately if not waiting
            close(sock);
        }
    }
    
    if (wait_for_reboot) {
        // Wait 10 seconds for all drones to reboot when explicitly requested
        std::cout << "All reboot commands sent. Waiting 10 seconds for drones to reboot...\n";
        std::this_thread::sleep_for(std::chrono::seconds(10));
        
        // Close all sockets
        for (int sock : sockets) {
            close(sock);
        }
        
        std::cout << "All drones rebooted.\n";
    } else {
        std::cout << "All reboot commands sent.\n";
    }
}

// Function to handle cleanup on program exit
void cleanup() {
    std::cout << "Cleaning up before exit...\n";
    // Don't wait for reboot when exiting, but send land command first
    rebootAllDrones(false, true);
}

// Function to handle a flight with a selected drone
bool handleFlight(const std::string& drone_ip) {
    if (drone_ip.empty()) {
        LOG_ERROR("No drone selected");
        return false;
    }
    
    LOG_INFO("Preparing flight for drone IP: " + drone_ip);
    
    // Find the optitrack_name for this drone IP
    std::string optitrack_name = "Bird1"; // Default
    std::string drone_id = "unknown";
    for (const auto& drone : allDrones) {
        if (drone.ip == drone_ip) {
            optitrack_name = drone.optitrack_name;
            drone_id = drone.id;
            break;
        }
    }
    
    LOG_INFO("Selected drone ID: " + drone_id + ", IP: " + drone_ip + ", OptiTrack name: " + optitrack_name);
    
    // Check if OptiTrack can see this tracker before flight
    bool tracker_visible = false;
    double position_x = get_optitrack_x_for_name(optitrack_name);
    double position_y = get_optitrack_y_for_name(optitrack_name);
    double position_yaw = get_optitrack_yaw_for_name(optitrack_name);
    
    if (position_x != 0.0 || position_y != 0.0) {
        tracker_visible = true;
        LOG_INFO("OptiTrack tracking confirmed for " + optitrack_name + 
                 ". Position: x=" + std::to_string(position_x) + 
                 ", y=" + std::to_string(position_y) + 
                 ", yaw=" + std::to_string(position_yaw));
    } else {
        LOG_WARNING("OptiTrack cannot see tracker " + optitrack_name + 
                  ". Drone position may not be tracked correctly!");
    }
    
    LOG_INFO("Creating drone control object for " + drone_ip);
    DroneControl drone(drone_ip, optitrack_name);
    
    LOG_INFO("Starting flight sequence for drone " + drone_id + " (" + drone_ip + ")");
    bool flightSuccess = drone.fly_and_log();
    
    if (flightSuccess) {
        LOG_INFO("Flight completed successfully for drone " + drone_id);
        // Only record data if we have flight data
        if (drone.hasFlightData()) {
            LOG_INFO("Flight data recorded successfully to flight_data.csv");
        } else {
            LOG_WARNING("No flight data to record for drone " + drone_id);
        }
    } else {
        LOG_ERROR("Flight failed for drone " + drone_id);
    }
    
    return flightSuccess;
}

// Display menu and handle user input
void displayMenu() {
    if (allDrones.empty()) {
        std::cout << "No drones found. Using default drone IP.\n";
        allDrones.push_back({"default", "192.168.10.1"});
    }
    
    std::cout << "\n===== Drone Control Menu =====\n";
    
    // Display drone list
    for (size_t i = 0; i < allDrones.size(); ++i) {
        std::cout << i + 1 << ". Drone ID: " << allDrones[i].id
                  << " (IP: " << allDrones[i].ip 
                  << ", OptiTrack: " << allDrones[i].optitrack_name << ")\n";
    }
    
    // Display options
    std::cout << "R. Reboot all drones\n";
    std::cout << "Q. Quit (and reboot all drones)\n";
    std::cout << "Enter choice (1-" << allDrones.size() << ", R, Q): ";
    
    std::string input;
    std::cin >> input;
    
    // Convert input to uppercase if it's a letter
    if (input.length() == 1 && std::isalpha(input[0])) {
        input[0] = std::toupper(input[0]);
    }
    
    // Handle menu options
    if (input == "R") {
        // When explicitly requesting reboot, wait for it to complete but don't land first
        rebootAllDrones(true, false);
    } else if (input == "Q") {
        std::cout << "Exiting program...\n";
        exit(0); // This will trigger the atexit handler
    } else {
        // Try to convert input to number for drone selection
        try {
            int choice = std::stoi(input);
            if (choice >= 1 && choice <= static_cast<int>(allDrones.size())) {
                // Valid drone selection
                handleFlight(allDrones[choice - 1].ip);
            } else {
                std::cerr << "Invalid selection. Please try again.\n";
            }
        } catch (const std::exception& e) {
            std::cerr << "Invalid input. Please try again.\n";
        }
    }
}

int main() {
    LOG_INFO("Starting drone control program");
    
    // Register cleanup function to be called at program exit
    atexit(cleanup);
    LOG_DEBUG("Registered cleanup handler");
    
    // Load drones from JSON file
    LOG_INFO("Loading drones from dji_devices.json");
    allDrones = loadDronesFromJSON("dji_devices.json");
    
    // If no drones found, set up a default drone
    if (allDrones.empty()) {
        LOG_WARNING("No drones found in dji_devices.json, using default");
        allDrones.push_back({"default", "192.168.10.1"});
    } else {
        LOG_INFO("Loaded " + std::to_string(allDrones.size()) + " drones");
    }
    
    // Initialize visualization
    LOG_INFO("Initializing OptiTrack visualization");
    init_optitrack_viz();
    LOG_INFO("OptiTrack visualization initialized");
    
    // Main menu loop
    LOG_INFO("Entering main menu loop");
    while (true) {
        displayMenu();
    }
    
    return 0;
}