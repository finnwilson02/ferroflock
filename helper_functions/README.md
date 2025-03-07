# FerroFlock Helper Functions

A comprehensive library for tracking and controlling multiple drones using OptiTrack motion capture system, with tools for data logging, calibration, and visualization.

## Overview

This project provides a set of helper functions and classes for:
- Interfacing with OptiTrack motion capture systems
- Real-time drone tracking and visualization
- Data logging with CSV export
- Drone control via Tello interface
- Motion tracking calibration
- Interactive menu-based user interface

## Prerequisites

- CMake (3.10 or newer)
- C++17 compatible compiler
- OpenCV 4
- VRPN library (for OptiTrack)
- Quaternion library
- Network connectivity to OptiTrack system
- Tello drones (for control functionality)

## Project Structure

```
ferroflock/helper_functions/
├── build/             # Build output directory
├── data/              # Logged data output
├── drone_info/        # Drone configuration files
├── include/           # Header files
│   ├── calibration.h  # Calibration utilities
│   ├── logger.h       # Data logging
│   ├── menu.h         # User interface
│   ├── optitrack.h    # Motion capture interface
│   └── tello_controller.h # Drone control
├── src/               # Source implementations
├── tests/             # Test scripts
│   └── test_optitrack_logging.cpp # OptiTrack logging test
├── python/            # Python utilities
├── CMakeLists.txt     # CMake build system
├── Makefile           # Direct make-based build system
└── README.md          # This documentation
```

## Key Components

### OptiTrack Interface

The `OptiTrack` class provides a reliable interface to the OptiTrack motion capture system:
- Real-time position and orientation tracking
- Yaw correction algorithms (handling 180° ambiguity)
- Visual representation of trackers with path history
- Tracker identification and management
- Active tracker detection and filtering

### Logger

The `Logger` class handles data recording for later analysis:
- CSV format logging with customizable fields
- Timestamped and unique file naming
- Position, orientation, and command data logging
- Thread-safe operation with mutex protection
- Auto-flush functionality to prevent data loss

### Tello Controller

The `TelloController` class interfaces with Tello drones:
- Connection management to multiple drones
- Command execution and queuing
- State tracking and telemetry
- Battery monitoring
- Emergency control features

### Calibration Tools

The `Calibration` class provides utilities for aligning coordinate systems:
- Frame calibration for global positioning
- Yaw offset calibration for correct orientation
- Automated and manual calibration workflows
- Reference point setting

### Menu System

The `Menu` class provides an interactive console interface:
- Main menu with multiple options
- Configuration management
- Calibration procedures
- Drone control interface
- Debug features

## Building the Project

### Using CMake (Recommended)

```bash
mkdir -p build
cd build
cmake ..
make
```

### Using Make

```bash
make           # Build the main executables
make tests     # Build the test scripts
make clean     # Clean build artifacts
```

### Command-line Options

When running the main application:
```
./build/helper_functions [options]

Options:
  -d, --debug    Enable debug output
  -h, --help     Display help message
```

## Running the Software

### Main Application

```bash
./build/helper_functions
```

This will launch the interactive menu system where you can:
- Configure drone settings
- Perform calibration
- Control drones
- Monitor drone positions
- Log data

### Testing OptiTrack Logging

```bash
./build/test_optitrack_logging
```

This standalone test script:
1. Connects to OptiTrack system
2. Identifies the first active tracker with non-zero position
3. Logs position and yaw data to a CSV file for 30 seconds or until Ctrl+C is pressed
4. Displays tracker information in real time
5. Creates a unique timestamped log file in the data/ directory

## Data Format

The logged CSV files include the following columns:
- Timestamp (in system time with millisecond precision)
- Tracker ID (string identifier)
- X, Y, Z positions (in meters)
- Quaternion orientation (qw, qx, qy, qz)
- Raw yaw angle (in radians, before correction)
- Corrected yaw angle (in radians, after flip correction)
- IMU yaw data (when available)
- Command data (when available)

## Development

### Creating New Tests

1. Place new test files in the `tests/` directory
2. Update the `CMakeLists.txt` file to include them:
   ```cmake
   set(NEW_TEST_SOURCES tests/your_new_test.cpp)
   add_executable(your_new_test ${NEW_TEST_SOURCES})
   target_link_libraries(your_new_test PRIVATE your_required_libs)
   ```
3. Add the test to the Makefile (optional but recommended):
   ```make
   $(BUILD_DIR)/your_new_test: $(TEST_DIR)/your_new_test.cpp $(OBJECTS)
       $(CC) $(CFLAGS) $(TEST_DIR)/your_new_test.cpp $(OBJECTS) -o $@ $(LDFLAGS)
   ```

For quick testing, use the `make tests` target:
```bash
make tests
```

### Common Tasks

#### Drone Configuration

Drone information is stored in JSON format. To add a new drone:
1. Update the drone configuration file with its information
2. Use the menu system to associate trackers with drones

#### Calibration Workflow

1. Run the main application
2. Select calibration from the menu
3. Follow the step-by-step guide to:
   - Set reference points
   - Adjust yaw offsets
   - Validate tracking accuracy

#### Tracker Visualization

The OptiTrack visualization provides:
- Real-time position display
- Path history visualization
- Orientation indicators
- Color-coded tracker identification

## Troubleshooting

- If OptiTrack connection fails:
  - Check network settings and ensure VRPN server is running
  - Verify the IP address of the OptiTrack server
  - Check that tracker names are correctly defined

- If visualization issues occur:
  - Verify OpenCV installation
  - Check display environment variables
  - Ensure visualization window is not obscured

- If data logging issues occur:
  - Check write permissions in the data directory
  - Verify disk space availability
  - Check CSV file format with a text editor

- If drone control fails:
  - Verify drone battery levels
  - Check WiFi connection to the drones
  - Ensure drones are in range and powered on

## Contributing

Contributions to the FerroFlock helper functions are welcome. Please follow the existing code style and add appropriate tests for new functionality.

## License

This project is licensed under [LICENSE INFORMATION]