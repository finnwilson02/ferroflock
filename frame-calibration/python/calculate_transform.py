#!/usr/bin/env python3
import csv
import numpy as np
from scipy.optimize import least_squares
import os
import matplotlib.pyplot as plt
from datetime import datetime

def normalize_angle(angle):
    """Normalize angle to [-180, 180] degrees."""
    return (angle + 180) % 360 - 180

def read_flight_data(csv_file='flight_data.csv'):
    """Read flight data from CSV file."""
    print(f"Reading data from {csv_file}...")
    try:
        with open(csv_file, 'r') as f:
            reader = csv.DictReader(f)
            timestamps = []
            commanded_yaws = []
            optitrack_yaws = []
            imu_yaws = []
            optitrack_x = []
            optitrack_y = []
            
            for row in reader:
                timestamps.append(float(row['timestamp']))
                commanded_yaws.append(float(row['commanded_yaw_deg']))
                optitrack_yaws.append(float(row['optitrack_yaw_deg']))
                imu_yaws.append(float(row['imu_yaw_deg']))
                optitrack_x.append(float(row['optitrack_x_m']))
                optitrack_y.append(float(row['optitrack_y_m']))
                
            print(f"Read {len(timestamps)} data points")
            return {
                'timestamps': np.array(timestamps),
                'commanded_yaws': np.array(commanded_yaws),
                'optitrack_yaws': np.array(optitrack_yaws),
                'imu_yaws': np.array(imu_yaws),
                'optitrack_x': np.array(optitrack_x),
                'optitrack_y': np.array(optitrack_y)
            }
    except Exception as e:
        print(f"Error reading flight data: {e}")
        return None

def residuals(transform, cmd_yaws, opt_yaws):
    """Residual function for least squares optimization.
    
    Models: optitrack_yaw = commanded_yaw + transform
    Minimizes: sum([normalize_angle(opt_yaw - (cmd_yaw + transform))^2])
    """
    return [normalize_angle(opt_yaw - (cmd_yaw + transform[0])) for cmd_yaw, opt_yaw in zip(cmd_yaws, opt_yaws)]

def compute_transform_with_least_squares(data):
    """Compute transform using least squares fit.
    
    Args:
        data: Dictionary containing flight data
    
    Returns:
        Optimized transform angle in degrees
    """
    commanded_yaws = data['commanded_yaws']
    optitrack_yaws = data['optitrack_yaws']
    
    # Initial guess: mean difference
    initial_transform = np.mean([normalize_angle(oy - cy) for cy, oy in zip(commanded_yaws, optitrack_yaws)])
    print(f"Initial transform estimate (mean difference): {initial_transform:.2f} degrees")
    
    # Least-squares fit
    result = least_squares(residuals, [initial_transform], args=(commanded_yaws, optitrack_yaws))
    transform = result.x[0]
    
    # Calculate residuals for the optimized transform
    final_residuals = residuals(result.x, commanded_yaws, optitrack_yaws)
    residual_std = np.std(final_residuals)
    max_residual = np.max(np.abs(final_residuals))
    
    print(f"Optimization results:")
    print(f"  - Transform: {transform:.2f} degrees")
    print(f"  - Mean residual: {np.mean(final_residuals):.2f} degrees")
    print(f"  - Std dev of residuals: {residual_std:.2f} degrees")
    print(f"  - Max absolute residual: {max_residual:.2f} degrees")
    print(f"  - Optimization success: {result.success}")
    
    return transform

def save_transform(transform, csv_file='transform.csv'):
    """Save transform to CSV file."""
    with open(csv_file, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['transform_angle_deg', 'timestamp'])
        writer.writerow([transform, datetime.now().isoformat()])
    
    print(f"Transform saved to {csv_file}")

def plot_results(data, transform):
    """Plot results to visualize the fit."""
    timestamps = data['timestamps'] - data['timestamps'][0]  # Relative time
    commanded_yaws = data['commanded_yaws']
    optitrack_yaws = data['optitrack_yaws']
    imu_yaws = data['imu_yaws']
    
    # Calculate expected OptiTrack values using the transform
    expected_optitrack = commanded_yaws + transform
    
    plt.figure(figsize=(10, 6))
    
    # Plot raw data
    plt.subplot(2, 1, 1)
    plt.plot(timestamps, commanded_yaws, 'b-', label='Commanded Yaw')
    plt.plot(timestamps, optitrack_yaws, 'r-', label='OptiTrack Yaw')
    plt.plot(timestamps, imu_yaws, 'g-', label='IMU Yaw')
    plt.plot(timestamps, expected_optitrack, 'k--', label=f'Commanded + Transform ({transform:.2f}°)')
    plt.xlabel('Time (s)')
    plt.ylabel('Yaw Angle (deg)')
    plt.legend()
    plt.title('Yaw Angle vs Time')
    plt.grid(True)
    
    # Plot residuals
    plt.subplot(2, 1, 2)
    residuals_array = np.array([normalize_angle(oy - (cy + transform)) 
                               for cy, oy in zip(commanded_yaws, optitrack_yaws)])
    plt.plot(timestamps, residuals_array, 'r-')
    plt.axhline(y=0, color='k', linestyle='--')
    plt.xlabel('Time (s)')
    plt.ylabel('Residual (deg)')
    plt.title('Residuals (OptiTrack - (Commanded + Transform))')
    plt.grid(True)
    
    plt.tight_layout()
    plt.savefig('transform_plot.png')
    print("Plot saved to transform_plot.png")
    
    # Optionally display the plot
    # plt.show()

def main():
    """Main function to process flight data and compute transform."""
    # Read flight data
    data = read_flight_data()
    if data is None or len(data['timestamps']) == 0:
        print("No data found. Make sure flight_data.csv exists and contains data.")
        return
    
    # Compute transform
    transform = compute_transform_with_least_squares(data)
    
    # Save transform
    save_transform(transform)
    
    # Plot results
    plot_results(data, transform)
    
    print(f"\nFinal yaw transform: {transform:.2f} degrees")
    print("This means: optitrack_yaw = commanded_yaw + transform")
    print("To convert from commanded to OptiTrack frame: ADD the transform")
    print("To convert from OptiTrack to commanded frame: SUBTRACT the transform")

if __name__ == "__main__":
    main()