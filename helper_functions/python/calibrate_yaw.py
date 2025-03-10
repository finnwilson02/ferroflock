#!/usr/bin/env python3
"""
calibrate_yaw.py - Utility to calibrate yaw offset between drone frame and OptiTrack frame

Purpose: Process CSV log files to calculate the yaw offset for each drone tracker
based on position data from flight tests, and update the dji_devices.json with the calibration.

Data Flow:
  Input: CSV file from ../data/ containing tracker positions and orientation data
  Output: Updated ~/ferroflock/dji_devices.json with calculated yaw_offset values
"""

import os
import sys
import json
import pandas as pd
import numpy as np
from datetime import datetime
import matplotlib.pyplot as plt

def list_csv_files():
    """List all CSV files in the data directory."""
    data_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "data")
    
    if not os.path.exists(data_dir):
        print(f"Error: Data directory {data_dir} does not exist.")
        sys.exit(1)
    
    csv_files = [f for f in os.listdir(data_dir) if f.endswith(".csv")]
    
    if not csv_files:
        print(f"No CSV files found in {data_dir}")
        sys.exit(1)
    
    # Sort by modification time (newest first)
    csv_files.sort(key=lambda f: os.path.getmtime(os.path.join(data_dir, f)), reverse=True)
    
    return csv_files, data_dir

def select_csv_file(csv_files):
    """Let the user select a CSV file from the list."""
    print("\nAvailable CSV files:")
    for i, file in enumerate(csv_files, 1):
        file_path = os.path.join("../data", file)
        mod_time = datetime.fromtimestamp(os.path.getmtime(file_path))
        print(f"{i}. {file} (Last modified: {mod_time.strftime('%Y-%m-%d %H:%M:%S')})")
    
    while True:
        try:
            choice = input("\nEnter the number of the CSV file to use: ")
            index = int(choice) - 1
            if 0 <= index < len(csv_files):
                return csv_files[index]
            else:
                print(f"Please enter a number between 1 and {len(csv_files)}.")
        except ValueError:
            print("Please enter a valid number.")

def circular_mean(angles_deg):
    """Calculate the circular mean of angles in degrees."""
    angles_rad = np.deg2rad(angles_deg)
    sum_sin = np.sum(np.sin(angles_rad))
    sum_cos = np.sum(np.cos(angles_rad))
    mean_rad = np.arctan2(sum_sin, sum_cos)
    mean_deg = np.rad2deg(mean_rad)
    return (mean_deg + 360) % 360  # Normalize to [0, 360)

def circular_variance(angles_deg):
    """Calculate the circular variance of angles in degrees."""
    angles_rad = np.deg2rad(angles_deg)
    sin_mean = np.mean(np.sin(angles_rad))
    cos_mean = np.mean(np.cos(angles_rad))
    r = np.sqrt(sin_mean**2 + cos_mean**2)
    return 1.0 - r  # Lower is better (less variance)

def normalize_yaw_offset(yaw_offset):
    """Normalize yaw offset to the range [-180, 180]."""
    if yaw_offset > 180:
        return yaw_offset - 360
    elif yaw_offset < -180:
        return yaw_offset + 360
    return yaw_offset

def visualize_paths(tracker_id, straight_motion, avg_yaw, yaw_offset, data_dir):
    """
    Visualize the actual, expected, and corrected paths of the drone in the XY plane.
    
    Parameters:
    - tracker_id (str): The ID of the tracker being visualized
    - straight_motion (pd.DataFrame): DataFrame with straight motion data (x, y, yaw_raw or yaw_corrected)
    - avg_yaw (float): Average yaw angle in degrees from circular_mean
    - yaw_offset (float): Calculated yaw offset in degrees
    - data_dir (str): Directory to save the output PNG file
    """
    # Extract actual path from straight_motion
    x_actual = straight_motion['x'].values
    y_actual = straight_motion['y'].values
    
    # Use the first point as the starting point for all paths
    start_x, start_y = x_actual[0], y_actual[0]
    
    # Determine which yaw field to use
    yaw_field = 'yaw_corrected' if 'yaw_corrected' in straight_motion.columns else 'yaw_raw'
    initial_yaw = straight_motion[yaw_field].values[0]  # Use first yaw value for simplicity
    
    # Define path length (arbitrary, for visualization)
    path_length = 1.0  # meters
    
    # Calculate endpoints for expected path (initial yaw)
    expected_yaw_rad = np.deg2rad(initial_yaw)
    end_x_expected = start_x + path_length * np.cos(expected_yaw_rad)
    end_y_expected = start_y + path_length * np.sin(expected_yaw_rad)
    
    # Calculate endpoints for corrected path (initial yaw - yaw_offset)
    corrected_yaw = initial_yaw - yaw_offset
    corrected_yaw_rad = np.deg2rad(corrected_yaw)
    end_x_corrected = start_x + path_length * np.cos(corrected_yaw_rad)
    end_y_corrected = start_y + path_length * np.sin(corrected_yaw_rad)
    
    # Create the plot
    plt.figure(figsize=(10, 8))
    
    # Plot actual path
    plt.plot(x_actual, y_actual, 'b-', label='Actual Path', linewidth=2)
    
    # Plot expected path
    plt.plot([start_x, end_x_expected], [start_y, end_y_expected], 'r--', 
            label=f'Expected Path (Yaw: {initial_yaw:.1f}°)', linewidth=2)
    
    # Plot corrected path
    plt.plot([start_x, end_x_corrected], [start_y, end_y_corrected], 'g-.', 
            label=f'Corrected Path (Yaw: {corrected_yaw:.1f}°)', linewidth=2)
    
    # Add starting point marker
    plt.plot(start_x, start_y, 'ko', label='Start Point')
    
    # Customize plot
    plt.xlabel('X Position (m)')
    plt.ylabel('Y Position (m)')
    plt.title(f'Drone Path Visualization for Tracker {tracker_id}')
    plt.legend()
    plt.grid(True)
    plt.axis('equal')  # Equal aspect ratio for accurate angles
    
    # Save the plot
    output_file = os.path.join(data_dir, f'path_visualization_{tracker_id}.png')
    plt.savefig(output_file)
    print(f"  Visualization saved to: {output_file}")
    
    # Close the plot to free memory
    plt.close()

def detect_straight_motion(group_df):
    """
    Detect periods of straight motion in the position data using sliding window approach.
    Returns the window with lowest angle variance, or None if no good window is found.
    """
    # Ensure data is sorted by time
    if 'timestamp' in group_df.columns:
        group_df['time'] = pd.to_datetime(group_df['timestamp'])
    else:
        print("Warning: No timestamp column found, using index for time ordering")
        group_df['time'] = pd.to_datetime(group_df.index)
    
    group_df = group_df.sort_values('time')
    
    # Filter out rows where position is zero (tracker not visible)
    valid_data = group_df[(group_df['x'] != 0) & (group_df['y'] != 0) & (group_df['z'] != 0)]
    
    if len(valid_data) < 5:
        print(f"  Warning: Not enough valid position data points ({len(valid_data)}), need at least 5")
        return None
        
    # Calculate direction vectors between consecutive points
    valid_data['dx'] = valid_data['x'].diff()
    valid_data['dy'] = valid_data['y'].diff()
    
    # Calculate direction angles
    valid_data['angle'] = np.arctan2(valid_data['dy'], valid_data['dx']) * 180 / np.pi
    valid_data['angle'] = (valid_data['angle'] + 360) % 360  # Normalize to [0, 360)
    
    # Drop first row which has NaN for diff values
    valid_data = valid_data.dropna(subset=['dx', 'dy', 'angle'])
    
    if len(valid_data) < 4:
        print(f"  Warning: Not enough valid motion data points ({len(valid_data)}), need at least 4")
        return None
    
    # Use sliding window approach
    window_size = pd.Timedelta(seconds=3)  # 3-second window
    step_size = pd.Timedelta(seconds=0.5)  # 0.5-second step
    
    start_time = valid_data['time'].min()
    end_time = valid_data['time'].max() - window_size
    
    best_window = None
    lowest_variance = float('inf')
    
    current_time = start_time
    while current_time <= end_time:
        window_end = current_time + window_size
        window_data = valid_data[(valid_data['time'] >= current_time) & (valid_data['time'] <= window_end)]
        
        if len(window_data) >= 5:  # Require at least 5 data points
            variance = circular_variance(window_data['angle'])
            
            if variance < lowest_variance:
                lowest_variance = variance
                best_window = window_data
        
        current_time += step_size
    
    if best_window is None:
        print("  Warning: Could not find a good window of straight motion")
        return None
    
    print(f"  Found straight motion window with {len(best_window)} points (variance: {lowest_variance:.4f})")
    return best_window

def load_json_file(json_path):
    """Load the JSON file or create a new structure if it doesn't exist."""
    if os.path.exists(json_path):
        try:
            with open(json_path, 'r') as f:
                data = json.load(f)
            
            # Check if the expected structure exists
            if "devices" not in data or not isinstance(data["devices"], list):
                print(f"Warning: Invalid structure in {json_path}, initializing with empty devices list")
                data = {"devices": []}
        except json.JSONDecodeError:
            print(f"Warning: Error parsing {json_path}, initializing with empty devices list")
            data = {"devices": []}
        except Exception as e:
            print(f"Warning: Error reading {json_path}: {e}")
            data = {"devices": []}
    else:
        print(f"Note: JSON file {json_path} does not exist, will create a new one")
        data = {"devices": []}
    
    return data

def main():
    # List all CSV files in the data directory
    csv_files, data_dir = list_csv_files()
    
    # Let the user select a CSV file
    selected_file = select_csv_file(csv_files)
    selected_path = os.path.join(data_dir, selected_file)
    
    print(f"\nProcessing: {selected_path}")
    
    # Read the CSV file
    try:
        df = pd.read_csv(selected_path)
        print(f"Successfully read CSV with {len(df)} rows")
    except Exception as e:
        print(f"Error reading CSV file: {e}")
        sys.exit(1)
    
    # Check if tracker_id column exists
    if 'tracker_id' not in df.columns:
        print("Error: CSV file does not have a 'tracker_id' column. Is this a proper calibration log?")
        print(f"Available columns: {', '.join(df.columns)}")
        sys.exit(1)
    
    # Filter out rows where tracker_id is null
    df = df[df['tracker_id'].notnull()]
    
    if len(df) == 0:
        print("Error: No valid tracker data in CSV file")
        sys.exit(1)
    
    # Check for essential columns
    required_columns = ['x', 'y', 'z', 'yaw_raw']
    missing_columns = [col for col in required_columns if col not in df.columns]
    
    if missing_columns:
        print(f"Error: Missing required columns: {', '.join(missing_columns)}")
        print(f"Available columns: {', '.join(df.columns)}")
        sys.exit(1)
    
    # Group by tracker_id
    tracker_groups = df.groupby('tracker_id')
    print(f"Found {len(tracker_groups)} tracker(s) in the data")
    
    # Calculate yaw offset for each tracker
    tracker_yaw_offsets = {}
    
    for tracker_id, group in tracker_groups:
        print(f"\nProcessing tracker: {tracker_id}")
        
        # Detect straight motion
        straight_motion = detect_straight_motion(group)
        
        if straight_motion is None:
            print(f"  Cannot calculate yaw offset for {tracker_id} - no good motion data")
            continue
        
        # Calculate yaw offset using circular mean during straight motion
        if 'yaw_corrected' in straight_motion.columns:
            yaw_values = straight_motion['yaw_corrected'].values
            yaw_field = 'yaw_corrected'
        else:
            yaw_values = straight_motion['yaw_raw'].values
            yaw_field = 'yaw_raw'
        
        # Calculate average yaw angle during straight motion
        avg_yaw = circular_mean(yaw_values)
        
        # Normalize to the range [-180, 180]
        yaw_offset = normalize_yaw_offset(avg_yaw)
        
        print(f"  Average {yaw_field}: {avg_yaw:.2f}° (normalized offset: {yaw_offset:.2f}°)")
        print(f"  Based on {len(yaw_values)} data points")
        
        # Store the result
        tracker_yaw_offsets[tracker_id] = yaw_offset
        
        # Visualize the paths
        if straight_motion is not None:
            visualize_paths(tracker_id, straight_motion, avg_yaw, yaw_offset, data_dir)
            print(f"  Please review the visualization for tracker {tracker_id} before proceeding.")
            input("  Press Enter to continue...")
    
    if not tracker_yaw_offsets:
        print("\nNo valid yaw offsets calculated. Cannot update configuration.")
        sys.exit(1)
    
    # Load the JSON file
    json_path = os.path.expanduser("~/ferroflock/dji_devices.json")
    temp_json_path = "./temp_dji_devices.json"
    
    drone_data = load_json_file(json_path)
    
    # Update the drones with calculated yaw offsets
    for tracker_id, yaw_offset in tracker_yaw_offsets.items():
        print(f"\nLooking for drone with tracker_id: {tracker_id}")
        
        # Find matching drone
        matching_drone = None
        for device in drone_data["devices"]:
            if device.get("tracker_id") == tracker_id:
                matching_drone = device
                break
        
        if matching_drone:
            # Display current information
            current_offset = matching_drone.get("yaw_offset", "not set")
            drone_name = matching_drone.get("name", "Unnamed")
            
            print(f"Found drone: {drone_name}")
            print(f"  Current yaw_offset: {current_offset}")
            print(f"  Calculated yaw_offset: {yaw_offset:.2f}°")
            
            # Ask for confirmation
            choice = input(f"\nSet yaw_offset to {yaw_offset:.2f}° for {tracker_id}? (y/n): ").strip().lower()
            
            if choice == 'y':
                # Update the yaw_offset
                matching_drone["yaw_offset"] = yaw_offset
                
                # Write to temp file first
                try:
                    with open(temp_json_path, 'w') as f:
                        json.dump(drone_data, f, indent=4)
                    
                    print(f"Written to temporary file: {temp_json_path}")
                    
                    # Copy to final location
                    result = os.system(f"cp {temp_json_path} {json_path}")
                    
                    if result == 0:
                        print(f"Successfully updated {json_path}")
                    else:
                        print(f"Error: Failed to copy to {json_path} (permission issue)")
                        print(f"Please manually copy {temp_json_path} to {json_path}")
                except Exception as e:
                    print(f"Error writing to JSON file: {e}")
            else:
                print("Update canceled")
        else:
            print(f"No device found with tracker_id {tracker_id} in {json_path}")
    
    print("\nCalibration process complete!")

if __name__ == "__main__":
    main()