#!/usr/bin/env python3
"""
Drone Control Interface - Python module for controlling the drone flight sequence.

This module provides a Python interface to the C++ drone control system. It allows:
1. Starting a calibration flight sequence
2. Monitoring drone status during flight
3. Processing flight data for yaw transform calculation
"""

import os
import subprocess
import time
import csv
import numpy as np
from datetime import datetime
from drone_state import DroneState

class DroneControlInterface:
    """Interface to the C++ drone control system."""
    
    def __init__(self, drone_id="Bird1"):
        """
        Initialize the drone control interface.
        
        Args:
            drone_id: The OptiTrack ID of the drone to control
        """
        self.drone_id = drone_id
        self.drone_state = DroneState()
        self.drone_control_binary = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            "build", "drone_control"
        )
        self.flight_data_csv = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            "flight_data.csv"
        )
    
    def start_calibration_flight(self):
        """
        Start a calibration flight sequence using the C++ drone control.
        
        Returns:
            True if the flight completed successfully, False otherwise
        """
        print(f"Starting calibration flight for drone {self.drone_id}")
        
        try:
            # Run the C++ drone control program
            result = subprocess.run(
                [self.drone_control_binary],
                check=True,
                text=True,
                stderr=subprocess.STDOUT
            )
            
            # If we get here, the flight completed successfully
            print("Flight completed successfully")
            return True
            
        except subprocess.CalledProcessError as e:
            print(f"Error during flight: {e}")
            return False
    
    def process_flight_data(self):
        """
        Process the flight data to calculate yaw transform.
        
        Returns:
            The computed yaw transform angle in degrees
        """
        print(f"Processing flight data from {self.flight_data_csv}")
        
        # Read the flight data CSV
        with open(self.flight_data_csv, 'r') as f:
            reader = csv.DictReader(f)
            data = list(reader)
        
        if not data:
            print("No flight data found")
            return None
        
        # Extract the data we need
        commanded_yaws = [float(row['commanded_yaw_deg']) for row in data]
        optitrack_yaws = [float(row['optitrack_yaw_deg']) for row in data]
        
        # Calculate the mean difference as the transform
        # This is a simplified version - the C++ calculate_transform.py uses
        # least squares fitting
        diff_angles = [self._normalize_angle(o - c) 
                       for o, c in zip(optitrack_yaws, commanded_yaws)]
        transform = np.mean(diff_angles)
        
        print(f"Computed yaw transform: {transform:.2f} degrees")
        return transform
    
    def _normalize_angle(self, angle):
        """Normalize angle to [-180, 180] degrees."""
        return (angle + 180) % 360 - 180
        
    def get_drone_state(self):
        """
        Get the current state of the drone.
        
        Returns:
            The DroneState object
        """
        # In a real implementation, this would update from OptiTrack or sensors
        # For now, we just return the current state object
        return self.drone_state

def initialize_drone():
    """Initialize the drone for flight."""
    print("Drone initialized")
    
def send_command(x, y, z):
    """
    Send movement command to the drone.
    
    Args:
        x: Forward movement (-1.0 to 1.0)
        y: Left/right movement (-1.0 to 1.0)
        z: Up/down movement (-1.0 to 1.0)
    """
    print(f"Command sent: ({x}, {y}, {z})")

if __name__ == "__main__":
    # Simple example usage
    controller = DroneControlInterface()
    
    # Initialize the drone
    initialize_drone()
    
    # Start a calibration flight
    print("Would you like to start a calibration flight? (y/n)")
    if input().lower() == 'y':
        controller.start_calibration_flight()
        
        # After flight, process the data
        transform = controller.process_flight_data()
        if transform is not None:
            print(f"You can apply this transform in your drone code:")
            print(f"adjusted_yaw = commanded_yaw - {transform:.2f}")
    else:
        # Just demonstrate some basic commands
        send_command(0.5, 0.0, 0.0)  # Forward at half speed
        time.sleep(0.5)
        send_command(0.0, 0.0, 0.0)  # Stop