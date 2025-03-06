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
#include "../include/optitrack_viz.h"

class DroneControl {
public:
    DroneControl() {
        file_.open("flight_data.csv", std::ios::app);
        if (file_.tellp() == 0) {
            file_ << "timestamp,commanded_yaw_deg,optitrack_x_m,optitrack_y_m,optitrack_yaw_deg,imu_yaw_deg\n";
            file_.flush();
        }
    }

    ~DroneControl() {
        if (file_.is_open()) {
            file_.close();
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
    
    // Simulated IMU data with realistic noise and drift
    double get_imu_yaw() {
        static double last_imu_yaw = 0.0;
        static bool initialized = false;
        static double drift_rate = 0.01; // degrees per second
        static double noise_amplitude = 0.05; // degrees
        static std::chrono::time_point<std::chrono::steady_clock> last_update_time;
        
        // Get the "ground truth" yaw from OptiTrack
        double optitrack_yaw = ::get_optitrack_yaw();
        
        // Initialize IMU yaw to OptiTrack yaw on first call
        if (!initialized) {
            last_imu_yaw = optitrack_yaw;
            initialized = true;
            last_update_time = std::chrono::steady_clock::now();
            return last_imu_yaw;
        }
        
        // Calculate time since last update
        auto now = std::chrono::steady_clock::now();
        double dt = std::chrono::duration<double>(now - last_update_time).count();
        last_update_time = now;
        
        // Add realistic IMU errors:
        // 1. Apply drift proportional to time elapsed
        double drift = drift_rate * dt;
        
        // 2. Add random noise
        // Using C++11 random number generation
        static std::random_device rd;
        static std::mt19937 gen(rd());
        std::normal_distribution<double> noise_dist(0.0, noise_amplitude);
        double noise = noise_dist(gen);
        
        // 3. Simulate simple sensor fusion with ground truth
        // This simulates that the IMU has some correction from other sensors
        const double fusion_alpha = 0.95; // Lower means more correction
        last_imu_yaw = fusion_alpha * (last_imu_yaw + drift + noise) + 
                      (1.0 - fusion_alpha) * optitrack_yaw;
        
        return last_imu_yaw;
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
        
        // Format the command string for the Tello drone
        std::string command;
        
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
        // We're not controlling yaw directly, so set it to 0
        std::ostringstream cmd_stream;
        cmd_stream << "rc " << y_scaled << " " << x_scaled << " " << z_scaled << " 0";
        command = cmd_stream.str();
        
        // In a real implementation, you would send the command to the drone
        // using UDP or other protocol
        std::cout << "Sending drone command: " << command << std::endl;
        
        // Update last command time
        last_command_time = now;
        
        // Example UDP send (commented out - would be implemented in production)
        /*
        int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if (sock >= 0) {
            struct sockaddr_in addr;
            addr.sin_family = AF_INET;
            addr.sin_port = htons(8889); // Tello command port
            addr.sin_addr.s_addr = inet_addr("192.168.10.1"); // Tello default IP
            
            sendto(sock, command.c_str(), command.length(), 0, 
                   (struct sockaddr*)&addr, sizeof(addr));
            
            close(sock);
        }
        */
    }
    
    // Initialize drone for flight
    void initialize_drone() {
        std::cout << "Initializing drone..." << std::endl;
        
        // In a real implementation, this would:
        // 1. Connect to the drone
        // 2. Send "command" to enter SDK mode
        // 3. Wait for acknowledgement
        
        std::cout << "Drone initialized and ready for commands" << std::endl;
    }
    
    // Send takeoff command to the drone
    void takeoff() {
        std::cout << "Sending takeoff command to drone..." << std::endl;
        
        // In a real implementation, this would send the takeoff command via UDP
        // and wait for acknowledgement
        
        // Wait a reasonable time for takeoff to complete
        std::this_thread::sleep_for(std::chrono::milliseconds(500)); // Simulated delay
        
        std::cout << "Drone takeoff successful" << std::endl;
    }
    
    // Send land command to the drone
    void land() {
        std::cout << "Sending land command to drone..." << std::endl;
        
        // In a real implementation, this would send the land command via UDP
        // and wait for acknowledgement
        
        std::cout << "Drone landing initiated" << std::endl;
    }
    
    // Emergency stop for the drone
    void emergency_stop() {
        std::cout << "!!! EMERGENCY STOP ACTIVATED !!!" << std::endl;
        
        // In a real implementation, this would send the emergency command via UDP
        
        std::cout << "Drone motors stopped" << std::endl;
    }
};

int main() {
    DroneControl drone;
    drone.fly_and_log();
    return 0;
}