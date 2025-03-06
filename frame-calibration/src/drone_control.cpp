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
#include "../include/optitrack_viz.h"

// Forward declarations
void init_optitrack_viz();
void update_optitrack_viz(double x, double y, double yaw);
void stop_visualization();

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
        file_.open("flight_data.csv", std::ios::app);
        if (file_.tellp() == 0) {
            file_ << "timestamp,commanded_yaw_deg,optitrack_x_m,optitrack_y_m,optitrack_yaw_deg,imu_yaw_deg\n";
            file_.flush();
        }
        command_sock_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if (command_sock_ < 0) {
            std::cerr << "Failed to create command socket" << std::endl;
        }
        state_receiver_thread_ = std::thread(&DroneControl::state_receiver, this);
    }

    ~DroneControl() {
        if (file_.is_open()) {
            file_.close();
        }
        
        stop_state_receiver_ = true;
        if (state_receiver_thread_.joinable()) {
            state_receiver_thread_.join();
        }
        if (command_sock_ >= 0) {
            close(command_sock_);
        }
        
        // Clean up visualization
        stop_visualization();
    }

    void log(double timestamp, double cmd_yaw, double opt_x, double opt_y, double opt_yaw, double imu_yaw) {
        if (file_.is_open()) {
            file_ << std::fixed << std::setprecision(6) << timestamp << "," << cmd_yaw << ","
                  << opt_x << "," << opt_y << "," << opt_yaw << "," << imu_yaw << "\n";
            if (++write_count_ % 10 == 0) file_.flush();
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
        // Set up the flight timeline
        double timestamp = get_time();
        double takeoff_end = timestamp + 2.0;  // 2s takeoff
        double up_end = takeoff_end + 2.0;     // 2s upward
        double forward_start = up_end;         // Forward starts
        double forward_end = forward_start + 2.0; // 2s forward
        double flight_end = forward_end + 1.0; // +1s after
        
        // Create the flight data directory if it doesn't exist
        std::string data_dir = "data";
        std::filesystem::create_directories(data_dir);

        std::cout << "Starting flight sequence..." << std::endl;
        std::cout << "Takeoff: 0-" << takeoff_end << "s" << std::endl;
        std::cout << "Up flight: " << takeoff_end << "-" << up_end << "s" << std::endl;
        std::cout << "Forward flight: " << forward_start << "-" << forward_end << "s" << std::endl;
        std::cout << "Logging period: " << (forward_start - 1.0) << "-" << (forward_end + 1.0) << "s" << std::endl;
        std::cout << "Press Ctrl+C to abort flight (emergency stop)" << std::endl;

        // Emergency handler setup - static to be accessible from signal handler
        static DroneControl* emergency_instance = this;
        
        // Set up a signal handler for emergency stop
        std::signal(SIGINT, [](int signal) {
            std::cout << "Received abort signal!" << std::endl;
            if (emergency_instance) {
                emergency_instance->emergency_stop();
            }
            std::exit(signal);
        });
        
        try {
            // Main flight loop
            while ((timestamp = get_time()) < flight_end) {
                double cmd_yaw = 0.0; // Forward = 0°
                double opt_x = get_optitrack_x();
                double opt_y = get_optitrack_y();
                double opt_yaw = get_optitrack_yaw();
                double imu_yaw = get_imu_yaw();

                // Fly drone according to the current flight phase
                if (timestamp < takeoff_end) {
                    send_command(0.0, 0.0, 1.0); // Takeoff (z up)
                    std::cout << "Takeoff phase: " << (takeoff_end - timestamp) << "s remaining" << std::endl;
                } else if (timestamp < up_end) {
                    send_command(0.0, 0.0, 1.0); // Up
                    std::cout << "Up phase: " << (up_end - timestamp) << "s remaining" << std::endl;
                } else {
                    send_command(1.0, 0.0, 0.0); // Forward (x forward)
                    if (timestamp < forward_end) {
                        std::cout << "Forward phase: " << (forward_end - timestamp) << "s remaining" << std::endl;
                    } else {
                        std::cout << "Extra logging period: " << (flight_end - timestamp) << "s remaining" << std::endl;
                    }
                }

                // Log data during forward phase ±1s
                if (timestamp >= forward_start - 1.0 && timestamp <= forward_end + 1.0) {
                    log(timestamp, cmd_yaw, opt_x, opt_y, opt_yaw, imu_yaw);
                }

                // Update visualization
                update_optitrack_viz(opt_x, opt_y, opt_yaw);
                
                // Maintain 10Hz control rate
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
            
            // Flight sequence completed, now land the drone
            land();
            
            std::cout << "Flight sequence completed. Data logged to flight_data.csv" << std::endl;
            return true;
            
        } catch (const std::exception& e) {
            // If any exception occurs, attempt emergency landing
            std::cerr << "ERROR during flight: " << e.what() << std::endl;
            emergency_stop();
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

    // Custom functions to get OptiTrack data for this specific drone
    double get_optitrack_x() { 
        // Forward declare these functions from the optitrack_viz.cpp file
        extern double get_optitrack_x_for_name(const std::string& name);
        extern double get_optitrack_x();
        
        // Try to get data for this drone's specific OptiTrack name
        double result = get_optitrack_x_for_name(optitrack_name_);
        
        // If no valid data, fall back to default method
        if (result == 0.0) {
            return ::get_optitrack_x();
        }
        return result;
    }
    
    double get_optitrack_y() { 
        // Forward declare these functions
        extern double get_optitrack_y_for_name(const std::string& name);
        extern double get_optitrack_y();
        
        // Try to get data for this drone's specific OptiTrack name
        double result = get_optitrack_y_for_name(optitrack_name_);
        
        // If no valid data, fall back to default method
        if (result == 0.0) {
            return ::get_optitrack_y();
        }
        return result;
    }
    
    double get_optitrack_yaw() { 
        // Forward declare these functions
        extern double get_optitrack_yaw_for_name(const std::string& name);
        extern double get_optitrack_yaw();
        
        // Try to get data for this drone's specific OptiTrack name
        double result = get_optitrack_yaw_for_name(optitrack_name_);
        
        // If no valid data, fall back to default method
        if (result == 0.0) {
            return ::get_optitrack_yaw();
        }
        return result;
    }
    
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
        std::cout << "Sent command: " << command << std::endl;
    }
    
    // Initialize drone for flight
    void initialize_drone() {
        std::cout << "Initializing drone..." << std::endl;
        send_command_to_drone("command");
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        std::cout << "Drone initialized and ready for commands" << std::endl;
    }
    
    // Send takeoff command to the drone
    void takeoff() {
        std::cout << "Sending takeoff command to drone..." << std::endl;
        send_command_to_drone("takeoff");
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        std::cout << "Drone takeoff successful" << std::endl;
    }
    
    // Send land command to the drone
    void land() {
        std::cout << "Sending land command to drone..." << std::endl;
        send_command_to_drone("land");
        std::cout << "Drone landing initiated" << std::endl;
    }
    
    // Emergency stop for the drone
    void emergency_stop() {
        std::cout << "!!! EMERGENCY STOP ACTIVATED !!!" << std::endl;
        send_command_to_drone("emergency");
        std::cout << "Drone motors stopped" << std::endl;
    }
};

// Function to reboot all drones
void rebootAllDrones() {
    std::cout << "Rebooting all drones...\n";
    
    if (allDrones.empty()) {
        std::cout << "No drones found to reboot.\n";
        return;
    }
    
    // Create sockets and send reboot commands to all drones simultaneously
    std::vector<int> sockets;
    for (const auto& drone : allDrones) {
        std::cout << "Sending reboot command to drone " << drone.id 
                  << " (IP: " << drone.ip 
                  << ", OptiTrack: " << drone.optitrack_name << ")\n";
        
        // Create socket for this drone
        int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if (sock < 0) {
            std::cerr << "Failed to create socket for drone " << drone.id << std::endl;
            continue;
        }
        
        // Send reboot command
        struct sockaddr_in addr;
        addr.sin_family = AF_INET;
        addr.sin_port = htons(8889); // Tello command port
        addr.sin_addr.s_addr = inet_addr(drone.ip.c_str());
        
        std::string command = "reboot";
        sendto(sock, command.c_str(), command.length(), 0, 
               (struct sockaddr*)&addr, sizeof(addr));
        
        // Save socket for cleanup
        sockets.push_back(sock);
    }
    
    // Wait 10 seconds for all drones to reboot
    std::cout << "All reboot commands sent. Waiting 10 seconds for drones to reboot...\n";
    std::this_thread::sleep_for(std::chrono::seconds(10));
    
    // Close all sockets
    for (int sock : sockets) {
        close(sock);
    }
    
    std::cout << "All drones rebooted.\n";
}

// Function to handle cleanup on program exit
void cleanup() {
    std::cout << "Cleaning up before exit...\n";
    rebootAllDrones();
}

// Function to handle a flight with a selected drone
bool handleFlight(const std::string& drone_ip) {
    if (drone_ip.empty()) {
        std::cerr << "No drone selected\n";
        return false;
    }
    
    // Find the optitrack_name for this drone IP
    std::string optitrack_name = "Bird1"; // Default
    for (const auto& drone : allDrones) {
        if (drone.ip == drone_ip) {
            optitrack_name = drone.optitrack_name;
            break;
        }
    }
    
    std::cout << "Selected drone IP: " << drone_ip << " (OptiTrack: " << optitrack_name << ")" << std::endl;
    DroneControl drone(drone_ip, optitrack_name);
    bool flightSuccess = drone.fly_and_log();
    
    if (flightSuccess) {
        std::cout << "Flight completed successfully\n";
        // Only record data if we have flight data
        if (drone.hasFlightData()) {
            std::cout << "Flight data recorded successfully\n";
        } else {
            std::cout << "No flight data to record\n";
        }
    } else {
        std::cout << "Flight failed\n";
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
        rebootAllDrones();
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
    // Register cleanup function to be called at program exit
    atexit(cleanup);
    
    // Load drones from JSON file
    allDrones = loadDronesFromJSON("dji_devices.json");
    
    // If no drones found, set up a default drone
    if (allDrones.empty()) {
        std::cerr << "No drones found in dji_devices.json, using default\n";
        allDrones.push_back({"default", "192.168.10.1"});
    }
    
    // Initialize visualization
    init_optitrack_viz();
    
    // Main menu loop
    while (true) {
        displayMenu();
    }
    
    return 0;
}