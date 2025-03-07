# Frame Calibration System

This system helps calibrate the transform between a drone's command frame and the OptiTrack measurement frame. It consists of three main components:

1. C++ Drone Control - Flies the drone and logs data
2. OptiTrack Visualization - Displays the drone's position and orientation
3. Python Transform Calculator - Processes logged data to compute the yaw transform

## 1. Drone Control (C++)

The drone control component (`drone_control.cpp`) handles the flight sequence and data logging. It:

- Executes a predetermined flight pattern (takeoff, up, forward)
- Logs data to CSV during the forward flight phase (±1 second)
- Logs commanded yaw, OptiTrack measurements, and IMU readings

### Usage

Build and run:

```bash
cd build
cmake ..
make drone_control
./drone_control
```

This will:
1. Start the drone flight sequence
2. Log data to `flight_data.csv` during the forward flight phase
3. Provide visual feedback using the OptiTrack visualization

## 2. OptiTrack Visualization (C++)

The visualization component (`optitrack_viz.cpp` and `optitrack_viz.h`) provides a visual representation of the drone's position and orientation in the OptiTrack reference frame. This is integrated with the drone control module.

## 3. Transform Calculation (Python)

The transform calculator (`calculate_transform.py`) processes the logged flight data to compute the yaw transform using a least-squares fit. It:

- Reads flight data from `flight_data.csv`
- Calculates the transform using least-squares optimization
- Saves the result to `transform.csv`
- Generates a visualization plot (`transform_plot.png`)

### Usage

Run after collecting flight data:

```bash
python3 python/calculate_transform.py
```

The resulting transform represents the offset between the drone's command frame and the OptiTrack frame:

```
optitrack_yaw = commanded_yaw + transform
```

To convert:
- From commanded to OptiTrack frame: ADD the transform
- From OptiTrack to commanded frame: SUBTRACT the transform

## Complete Calibration Process

1. Run the drone control program:
   ```bash
   ./build/drone_control
   ```

2. Let the drone complete its flight sequence.

3. Process the logged data to compute the transform:
   ```bash
   python3 python/calculate_transform.py
   ```

4. Apply the computed transform in your drone control system.

## Data Files

- `flight_data.csv`: Raw flight data with timestamp, yaw measurements, and positions
- `transform.csv`: Computed yaw transform angle
- `transform_plot.png`: Visualization of the data and transform fit