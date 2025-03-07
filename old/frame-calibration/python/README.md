# Drone Frame Calibration

This module provides tools for calibrating the yaw angle transform between a drone's command frame and OptiTrack's measurement frame.

## Overview

The calibration system uses the following approach:

1. The drone is commanded to fly forward with a known yaw
2. OptiTrack measurements of the drone's yaw are recorded
3. The transform angle is computed as the average difference between commanded and measured yaw
4. The transform is refined iteratively over multiple runs

## Files

- `calibration_logger.py`: CSV logging system for calibration data
- `yaw_transform.py`: Functions for computing yaw transform angles
- `transform_refiner.py`: Tools for iterative refinement of transform angles
- `calibrate_frame.py`: Main script for performing calibration

## Usage

### Basic Calibration

Run a single calibration:

```bash
python calibrate_frame.py
```

This will:
- Perform one calibration run
- Collect 20 samples at 10 Hz
- Save the transform to the history file
- Update the refined transform

### Multiple Runs

Perform multiple calibration runs to refine the transform:

```bash
python calibrate_frame.py --runs 3
```

### Custom Parameters

Customize the calibration parameters:

```bash
python calibrate_frame.py --runs 2 --command-yaw 90.0 --samples 30 --rate 5.0
```

- `--runs`: Number of calibration runs to perform
- `--command-yaw`: The commanded "forward" yaw in degrees
- `--samples`: Number of samples to collect per run
- `--rate`: Sampling rate in Hz

### View Current Transform

View the current refined transform without running calibration:

```bash
python calibrate_frame.py --view-only
```

## Output Files

The system generates the following files in the `data` directory:

- `calibration_log_YYYYMMDD_HHMMSS.csv`: Detailed log of each calibration run
- `transform_history.csv`: History of all computed transforms

## Integration

To integrate the calibrated transform with your drone control system:

```python
# Get the current refined transform
from transform_refiner import get_refined_transform

transform = get_refined_transform()

# Apply the transform to commanded yaw
adjusted_yaw = commanded_yaw - transform
```

## Troubleshooting

- If the transform seems inconsistent, try increasing the number of samples
- Ensure the drone is flying a straight path during the sample collection
- For better results, perform multiple calibration runs