#include <fstream>
#include <string>
#include <chrono>
#include <iomanip>
#include <thread>
#include <iostream>
#include <vector>
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

    ~DroneControl() { if (file_.is_open()) file_.close(); }

    void log(double timestamp, double cmd_yaw, double opt_x, double opt_y, double opt_yaw, double imu_yaw) {
        if (file_.is_open()) {
            file_ << std::fixed << std::setprecision(6) << timestamp << "," << cmd_yaw << ","
                  << opt_x << "," << opt_y << "," << opt_yaw << "," << imu_yaw << "\n";
            if (++write_count_ % 10 == 0) file_.flush();
        }
    }

    void fly_and_log() {
        double timestamp = get_time();
        double takeoff_end = timestamp + 2.0;  // 2s takeoff
        double up_end = takeoff_end + 2.0;     // 2s upward
        double forward_start = up_end;         // Forward starts
        double forward_end = forward_start + 2.0; // 2s forward
        double flight_end = forward_end + 1.0; // +1s after

        std::cout << "Starting flight sequence..." << std::endl;
        std::cout << "Takeoff: 0-" << takeoff_end << "s" << std::endl;
        std::cout << "Up flight: " << takeoff_end << "-" << up_end << "s" << std::endl;
        std::cout << "Forward flight: " << forward_start << "-" << forward_end << "s" << std::endl;
        std::cout << "Logging period: " << (forward_start - 1.0) << "-" << (forward_end + 1.0) << "s" << std::endl;

        while ((timestamp = get_time()) < flight_end) {
            double cmd_yaw = 0.0; // Forward = 0°
            double opt_x = get_optitrack_x();  // Replace with your OptiTrack calls
            double opt_y = get_optitrack_y();
            double opt_yaw = get_optitrack_yaw();
            double imu_yaw = get_imu_yaw();    // Replace with your IMU call

            // Fly drone
            if (timestamp < takeoff_end) {
                send_command(0.0, 0.0, 1.0); // Takeoff (z up)
                std::cout << "Takeoff phase: " << (timestamp - (timestamp - takeoff_end)) << "s remaining" << std::endl;
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

            // Log during forward ±1s
            if (timestamp >= forward_start - 1.0 && timestamp <= forward_end + 1.0) {
                log(timestamp, cmd_yaw, opt_x, opt_y, opt_yaw, imu_yaw);
            }

            update_optitrack_viz(opt_x, opt_y, opt_yaw); // Viz call
            std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 10 Hz
        }
        
        std::cout << "Flight sequence completed. Data logged to flight_data.csv" << std::endl;
    }

private:
    std::ofstream file_;
    int write_count_ = 0;

    double get_time() { // Replace with your timing
        return std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count() / 1e6;
    }

    // Stubs - replace with your actual implementations
    double get_optitrack_x() { return 0.0; }
    double get_optitrack_y() { return 0.0; }
    double get_optitrack_yaw() { return 5.0; }  // Simulated value - normally from OptiTrack
    double get_imu_yaw() { return 4.8; }        // Simulated value - normally from IMU
    void send_command(double x, double y, double z) { 
        // Your drone command implementation
        std::cout << "Sending drone command: x=" << x << ", y=" << y << ", z=" << z << std::endl;
    }
};

int main() {
    DroneControl drone;
    drone.fly_and_log();
    return 0;
}