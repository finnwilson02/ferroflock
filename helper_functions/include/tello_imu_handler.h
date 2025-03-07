#ifndef TELLO_IMU_HANDLER_H
#define TELLO_IMU_HANDLER_H

#include "tello_controller.h"
#include <string>
#include <chrono>
#include <mutex>
#include <thread>
#include <optional>
#include <atomic>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <unistd.h>
#include <fcntl.h>

/**
 * @brief A modular handler for retrieving IMU data from Tello drones
 * 
 * This class provides a simple and efficient interface for retrieving IMU data
 * from Tello drones in station mode. It handles initialization, error checking,
 * and provides thread-safe access to IMU data.
 */
class TelloIMUHandler {
public:
    /**
     * @brief Construct a new TelloIMUHandler object
     * 
     * @param controller Reference to the TelloController managing drone connections
     * @param drone_ip IP address of the drone to retrieve IMU data from
     */
    TelloIMUHandler(TelloController& controller, const std::string& drone_ip);
    
    /**
     * @brief Destructor to clean up resources
     */
    ~TelloIMUHandler();
    
    /**
     * @brief Initialize the drone for IMU data reception
     * 
     * Sends the 'command' command to enter SDK mode and disables mission pad detection
     * 
     * @return true if initialization succeeded, false otherwise
     */
    bool initialize();
    
    /**
     * @brief Get the yaw angle from the IMU
     * @return double Yaw angle in degrees
     */
    double getYaw() const;
    
    /**
     * @brief Get the pitch angle from the IMU
     * @return double Pitch angle in degrees
     */
    double getPitch() const;
    
    /**
     * @brief Get the roll angle from the IMU
     * @return double Roll angle in degrees
     */
    double getRoll() const;
    
    /**
     * @brief Get the X-axis acceleration from the IMU
     * @return double X-axis acceleration in g (9.81 m/s²)
     */
    double getAgx() const;
    
    /**
     * @brief Get the Y-axis acceleration from the IMU
     * @return double Y-axis acceleration in g (9.81 m/s²)
     */
    double getAgy() const;
    
    /**
     * @brief Get the Z-axis acceleration from the IMU
     * @return double Z-axis acceleration in g (9.81 m/s²)
     */
    double getAgz() const;
    
    /**
     * @brief Check if the IMU data is valid
     * 
     * The data is considered valid if it has been updated within the last second.
     * 
     * @return true if data is valid, false otherwise
     */
    bool isDataValid() const;
    
    /**
     * @brief Get the IP address of the drone
     * @return const std::string& The IP address
     */
    const std::string& getIP() const { return ip; }

private:
    // Reference to the TelloController (for sending commands)
    TelloController& controller;
    // IP address of the drone
    std::string ip;
    
    // IMU data storage
    mutable std::mutex data_mutex;
    double imu_yaw{0.0};
    double imu_pitch{0.0};
    double imu_roll{0.0};
    double imu_agx{0.0};
    double imu_agy{0.0};
    double imu_agz{0.0};
    std::chrono::system_clock::time_point last_state_time;
    
    // Socket for state data
    int state_socket{-1};
    int state_port{8890};
    struct sockaddr_in state_addr{};
    bool socket_initialized{false};
    
    // Thread control
    std::atomic<bool> running{true};
    std::thread state_thread;
    
    // Internal methods
    bool initializeStateSocket();
    void stateReceiver();
    void parseStateData(const std::string& data);
    std::optional<std::string> receiveState(int timeout_ms = 100);
};

#endif // TELLO_IMU_HANDLER_H