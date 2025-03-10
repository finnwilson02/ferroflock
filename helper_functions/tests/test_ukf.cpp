#include "../include/ukf.h"
#include <iostream>
#include <fstream>
#include <Eigen/Dense>
#include <chrono>
#include <thread>

int main() {
    // Create UKF instance
    UKF ukf;
    
    // Load configuration
    std::cout << "Loading UKF configuration..." << std::endl;
    ukf.loadConfig("../config.json");
    
    // Simulate getting data at different rates
    const double predict_dt = 0.01;  // 100 Hz for predict
    const double opti_dt = 0.00556;  // 180 Hz for OptiTrack
    const double control_dt = 0.02;  // 50 Hz for control inputs
    
    // Initialize time
    double t = 0.0;
    double next_predict_time = 0.0;
    double next_opti_time = 0.0;
    double next_control_time = 0.0;
    
    // Create log file
    std::ofstream log_file("ukf_test_log.csv");
    log_file << "Time,x,y,z,vx,vy,vz,psi" << std::endl;
    
    // Simulate for 10 seconds
    while (t < 10.0) {
        // Execute predict at 100 Hz
        if (t >= next_predict_time) {
            ukf.predict(t);
            next_predict_time += predict_dt;
            
            // Log state
            Eigen::VectorXd state = ukf.getState();
            log_file << t << "," 
                    << state(0) << "," << state(1) << "," << state(2) << ","
                    << state(3) << "," << state(4) << "," << state(5) << ","
                    << state(6) << std::endl;
        }
        
        // Simulate OptiTrack data at 180 Hz
        if (t >= next_opti_time) {
            // Simulate simple linear motion with noise
            Eigen::Vector4d z_opti;
            z_opti(0) = 0.5 * t + 0.001 * (rand() % 1000 - 500) / 500.0;  // x
            z_opti(1) = 0.3 * t + 0.001 * (rand() % 1000 - 500) / 500.0;  // y
            z_opti(2) = 0.4 * sin(t) + 0.001 * (rand() % 1000 - 500) / 500.0;  // z
            z_opti(3) = 0.2 * t + 0.01 * (rand() % 1000 - 500) / 500.0;  // psi
            
            // Simulate occasional yaw flips (every 2 seconds)
            if (fmod(t, 2.0) <= 0.05 && fmod(t, 2.0) > 0) {
                z_opti(3) += M_PI;  // Add 180 degrees to simulate flip
            }
            
            // Simulate occasional dropouts (every 3 seconds)
            if (fmod(t, 3.0) > 0.5 || fmod(t, 3.0) <= 0) {
                ukf.update(t, z_opti);
            }
            
            next_opti_time += opti_dt;
        }
        
        // Simulate control inputs at 50 Hz
        if (t >= next_control_time) {
            // Generate simple control inputs
            Eigen::Vector4d u;
            u(0) = 0.2 * sin(t);     // rc_lr
            u(1) = 0.3 * cos(t);     // rc_fb
            u(2) = 0.1 * sin(2 * t); // rc_ud
            u(3) = 0.1 * cos(3 * t); // rc_yaw
            
            // Yaw rate from IMU (omega)
            double omega = 0.05 * sin(t);
            
            ukf.setControlInput(t, u, omega);
            next_control_time += control_dt;
        }
        
        // Increment time
        t += 0.001;  // 1 ms step for simulation
        
        // Slow down simulation to see it in real time
        std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
    
    std::cout << "UKF test completed. Results written to ukf_test_log.csv" << std::endl;
    log_file.close();
    
    return 0;
}