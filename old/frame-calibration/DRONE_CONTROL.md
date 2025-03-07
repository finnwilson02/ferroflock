# Tello Drone Control Implementation

This document explains the UDP-based drone control implementation for the Tello drone in the frame-calibration system.

## Overview

The drone control system (`src/drone_control.cpp`) implements UDP socket communication with Tello drones for command and state reception. It provides:

1. UDP command transmission on port 8889
2. UDP state reception on port 8890
3. Real-time IMU data processing
4. Predefined flight sequence execution
5. Flight data logging

## Communication Architecture

### Command Socket (Port 8889)

- Sends control commands to the Tello drone
- Commands include initialization, takeoff, landing, emergency stop, and movement control
- Communication is one-way (computer to drone)

### State Socket (Port 8890)

- Receives state information from the Tello drone
- Data includes IMU readings, battery levels, and other drone status information
- Communication is one-way (drone to computer)
- Runs in a dedicated thread to continuously process incoming data

## Key Components

### `DroneControl` Class

The main class that handles all drone interactions:

```cpp
class DroneControl {
public:
    DroneControl();  // Sets up sockets and state receiver thread
    ~DroneControl(); // Cleans up resources
    void log(...);   // Logs flight data
    void fly_and_log(); // Main flight sequence
    
private:
    // Socket management
    int command_sock_;
    std::string drone_ip_;
    int command_port_;
    
    // State reception
    double current_yaw_;
    std::thread state_receiver_thread_;
    bool stop_state_receiver_;
    
    // Helper methods
    void state_receiver();
    void parse_state_message(const std::string&);
    double get_imu_yaw();
    
    // Command methods
    void send_command(double x, double y, double z);
    void send_command_to_drone(const std::string&);
    void initialize_drone();
    void takeoff();
    void land();
    void emergency_stop();
};
```

### State Receiver Thread

A dedicated thread that continuously receives and processes state data:

```cpp
void state_receiver() {
    // Create UDP socket bound to port 8890
    // While running:
    //   Receive UDP packets
    //   Parse messages
    //   Extract yaw and other data
}
```

### Command Functions

The drone responds to several key commands:

- `command` - Enters SDK mode (must be sent first)
- `takeoff` - Initiates automatic takeoff
- `land` - Initiates automatic landing
- `emergency` - Emergency motor cutoff
- `rc a b c d` - Controls movement where:
  - `a`: left/right (-100 to 100)
  - `b`: forward/backward (-100 to 100)
  - `c`: up/down (-100 to 100)
  - `d`: yaw (-100 to 100)

## Flight Sequence

The system executes a predetermined flight sequence:

1. **Initialization** - Enters SDK mode
2. **Takeoff** - Automatic takeoff procedure (2 seconds)
3. **Up Movement** - Ascends vertically (2 seconds)
4. **Forward Movement** - Moves forward (2 seconds)
5. **Landing** - Automatic landing procedure

During the forward movement phase (±1 second), the system logs data including:
- Timestamp
- Commanded yaw
- OptiTrack position (x, y) and yaw
- IMU yaw

## Implementation Details

### Sending Commands

```cpp
void send_command_to_drone(const std::string& command) {
    // Format: Create socket address for drone
    // Send command string via UDP to 192.168.10.1:8889
    // Log the sent command
}
```

### Receiving State Data

```cpp
void parse_state_message(const std::string& message) {
    // Format: "pitch:0;roll:0;yaw:0;..."
    // Parse semicolon-separated values
    // Extract key-value pairs
    // Update current_yaw_ when yaw data is found
}
```

### Error Handling

The system includes several error handling mechanisms:

- Signal handling for emergency stop (SIGINT/Ctrl+C)
- Exception handling in the main flight loop
- Socket error detection and reporting
- Data parsing error handling

## Usage Notes

1. The drone must be powered on and broadcasting its Wi-Fi network
2. Your computer must be connected to the drone's Wi-Fi
3. The standard IP for Tello drones is 192.168.10.1
4. Commands are sent at approximately 10Hz
5. The state receiver operates asynchronously in its own thread
6. Socket cleanup is handled automatically in the destructor

## Integration with OptiTrack

The system integrates with OptiTrack for ground truth positioning:

- `get_optitrack_x()`, `get_optitrack_y()`, and `get_optitrack_yaw()` provide real-time position data
- This data is used both for logging and visualization
- Comparing OptiTrack yaw with IMU yaw enables calibration