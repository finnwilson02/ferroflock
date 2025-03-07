import numpy as np
import os
import csv
import time
from datetime import datetime
from calibration_logger import CalibrationLogger

def normalize_angle(angle):
    """
    Normalize angle to [-180, 180] degrees.
    
    Args:
        angle: Input angle in degrees
        
    Returns:
        Normalized angle in degrees within range [-180, 180]
    """
    return (angle + 180) % 360 - 180

def compute_transform(commanded_yaw, optitrack_yaws):
    """
    Compute yaw transform from commanded yaw and list of OptiTrack yaws.
    
    Args:
        commanded_yaw: Commanded yaw angle in degrees
        optitrack_yaws: List of OptiTrack measured yaw angles in degrees
        
    Returns:
        Transform angle in degrees
    """
    differences = [normalize_angle(yaw - commanded_yaw) for yaw in optitrack_yaws]
    return np.mean(differences)

def calibration_run(commanded_yaw=0.0, sample_count=20, sample_rate_hz=10.0):
    """
    Perform a single calibration run to collect yaw data and compute transform.
    
    Args:
        commanded_yaw: Forward commanded yaw angle in degrees (default: 0.0)
        sample_count: Number of samples to collect (default: 20)
        sample_rate_hz: Sampling rate in Hz (default: 10.0)
        
    Returns:
        Computed transform angle in degrees
    """
    logger = CalibrationLogger()
    transform_angle = 0.0  # Initial placeholder
    
    try:
        print(f"Starting calibration run with forward command yaw: {commanded_yaw} degrees")
        print(f"Collecting {sample_count} samples at {sample_rate_hz} Hz...")
        
        optitrack_yaws = []
        sample_interval = 1.0 / sample_rate_hz
        
        # Collect samples during straight segment
        for i in range(sample_count):
            # Replace this with actual OptiTrack yaw measurement
            # This is a simulated value - integrate with your actual OptiTrack data source
            optitrack_yaw = 5.0 + np.random.normal(0, 0.5)  # Simulated noisy measurement
            
            optitrack_yaws.append(optitrack_yaw)
            logger.log(commanded_yaw, optitrack_yaw, transform_angle)
            
            time.sleep(sample_interval)
            
        # Compute transform from collected data
        transform_angle = compute_transform(commanded_yaw, optitrack_yaws)
        print(f"Computed transform: {transform_angle:.2f} degrees")
        
        # Log final computed transform
        logger.log(commanded_yaw, optitrack_yaws[-1], transform_angle)
        
        return transform_angle
        
    finally:
        logger.close()

# Example usage (for testing)
if __name__ == "__main__":
    transform = calibration_run()
    print(f"Final transform angle: {transform:.2f} degrees")