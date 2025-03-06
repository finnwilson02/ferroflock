# Drone Controller

This program controls DJI Tello drones with OptiTrack tracking, visualizes the drone's position, and calculates the transform between the OptiTrack frame and command frame.

## Features

- OptiTrack tracking visualization with orientation arrows
- Drone path tracing in the visualization
- Flight sequence: takeoff, move up, move forward
- Transform calculation between OptiTrack and command frames
- Drone reboot functionality

## Prerequisites

- OpenCV library
- nlohmann_json library
- VRPN visualizer built in `/home/finn/squawkblock/squawkblock/vrpn/build/vrpn_viz`
- DJI Tello drones connected to the network
- OptiTrack motion capture system

## OptiTrack Mapping

The drones are mapped to OptiTrack trackers as follows:
- Bird 5 (tracker #8) - 192.168.1.107
- Bird 3 (tracker #6) - 192.168.1.106
- Bird 4 (tracker #7) - 192.168.1.104
- Bird 1 (tracker #4) - 192.168.1.108

## File Structure

```
drone_controller/
├── bin/            # Binary outputs
├── build/          # Build files
├── data/           # Configuration files
│   └── drone_mapping.json
├── include/        # Header files
│   ├── drone_controller.h
│   ├── drone_types.h
│   └── vrpn_client.h
├── src/            # Source files
│   ├── drone_controller.cpp
│   ├── main.cpp
│   └── vrpn_client.cpp
├── Makefile        # Build system
├── CMakeLists.txt  # CMake configuration
└── README.md       # Documentation
```

## Building

Using Make:
```bash
make
```

or using CMake:
```bash
mkdir -p build
cd build
cmake ..
make
```

## Usage

1. Run the program:
   ```
   ./bin/drone_controller
   ```

2. Select a drone from the menu
3. Enable visualization
4. Execute the flight sequence to:
   - Take off
   - Move upward for 2 seconds
   - Move forward for 5 seconds
   - Calculate the transform from OptiTrack frame to command frame
   - Land automatically

## Output

The program will calculate the transform from OptiTrack frame to command frame (in yaw only) and save it to `yaw_transform.txt`. The transform is also displayed in the visualization.

## Visualization

The visualization shows:
- The OptiTrack coordinate system with grid
- The positions of all tracked objects
- The orientation of drones (shown as arrows)
- The path traced by the drone during flight