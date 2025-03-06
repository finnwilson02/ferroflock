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
#include "../include/optitrack_viz.h"

class DroneControl {
public:
    DroneControl() {
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

    void fly_and_log() {
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
            
            // Wait a bit for the landing to complete
            std::this_thread::sleep_for(std::chrono::seconds(3));
            
        } catch (const std::exception& e) {
            // If any exception occurs, attempt emergency landing
            std::cerr << "ERROR during flight: " << e.what() << std::endl;
            emergency_stop();
        }
    }

private:
    std::ofstream file_;
    int write_count_ = 0;
    
    int command_sock_ = -1;              // Persistent socket for sending commands
    std::string drone_ip_ = "192.168.10.1"; // Tello IP
    int command_port_ = 8889;           // Tello command port
    double current_yaw_ = 0.0;          // Real-time IMU yaw from state messages
    std::thread state_receiver_thread_; // Thread for receiving state messages
    bool stop_state_receiver_ = false;  // Flag to stop the state receiver thread

    double get_time() { // Replace with your timing
        return std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count() / 1e6;
    }

    // OptiTrack functions implemented in optitrack_viz.cpp
    double get_optitrack_x() { 
        // This now uses the real OptiTrack tracking data
        return ::get_optitrack_x(); 
    }
    
    double get_optitrack_y() { 
        // This now uses the real OptiTrack tracking data
        return ::get_optitrack_y(); 
    }
    
    double get_optitrack_yaw() { 
        // This now uses the real OptiTrack tracking data
        return ::get_optitrack_yaw(); 
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
        
        // Scale inputs to appropriate ranges for Tello drones (-100 to 100)
        int x_scaled = static_cast<int>(x * 100);
        int y_scaled = static_cast<int>(y * 100);
        int z_scaled = static_cast<int>(z * 100);
        
        // Clamp values to valid range
        x_scaled = std::max(-100, std::min(100, x_scaled));
        y_scaled = std::max(-100, std::min(100, y_scaled));
        z_scaled = std::max(-100, std::min(100, z_scaled));
        
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

int main() {
    DroneControl drone;
    drone.fly_and_log();
    return 0;
}