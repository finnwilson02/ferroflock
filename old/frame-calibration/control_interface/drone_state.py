#!/usr/bin/env python3
"""
Drone State - Python module for tracking drone state information.
"""

import time
import math
from enum import Enum, auto

class FlightState(Enum):
    """Enumeration of possible drone flight states."""
    LANDED = auto()
    TAKING_OFF = auto()
    HOVERING = auto()
    FLYING = auto()
    LANDING = auto()
    EMERGENCY = auto()

class DroneState:
    """Class for tracking drone state information."""
    
    def __init__(self):
        """Initialize drone state with default values."""
        # Position and orientation
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.yaw = 0.0  # in degrees
        
        # Velocities
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.yaw_rate = 0.0  # in degrees/second
        
        # Flight state
        self.flight_state = FlightState.LANDED
        
        # Battery and connection
        self.battery_percentage = 100
        self.is_connected = False
        
        # Timestamps
        self.last_update_time = time.time()
        self.flight_time = 0.0  # in seconds
    
    def update_position(self, x, y, z):
        """
        Update position information.
        
        Args:
            x: X position in meters
            y: Y position in meters
            z: Z position in meters
        """
        now = time.time()
        dt = now - self.last_update_time
        
        # Calculate velocities if time has elapsed
        if dt > 0:
            self.vx = (x - self.x) / dt
            self.vy = (y - self.y) / dt
            self.vz = (z - self.z) / dt
        
        # Update position
        self.x = x
        self.y = y
        self.z = z
        
        # Update timestamps
        self.last_update_time = now
        if self.flight_state != FlightState.LANDED:
            self.flight_time += dt
        
        print(f"Position updated: ({self.x:.2f}, {self.y:.2f}, {self.z:.2f})")
    
    def update_yaw(self, yaw):
        """
        Update yaw orientation.
        
        Args:
            yaw: Yaw angle in degrees
        """
        now = time.time()
        dt = now - self.last_update_time
        
        # Normalize yaw to [-180, 180] degrees
        yaw = (yaw + 180) % 360 - 180
        
        # Calculate yaw rate if time has elapsed
        if dt > 0:
            # Calculate shortest angular distance, handling wrap-around
            diff = yaw - self.yaw
            if diff > 180:
                diff -= 360
            elif diff < -180:
                diff += 360
                
            self.yaw_rate = diff / dt
        
        # Update yaw
        self.yaw = yaw
        
        # Update timestamp
        self.last_update_time = now
    
    def set_flight_state(self, state):
        """
        Set the current flight state.
        
        Args:
            state: New flight state from FlightState enum
        """
        if state != self.flight_state:
            print(f"Flight state changed: {self.flight_state.name} -> {state.name}")
            self.flight_state = state
    
    def update_battery(self, percentage):
        """
        Update battery percentage.
        
        Args:
            percentage: Battery percentage (0-100)
        """
        self.battery_percentage = max(0, min(100, percentage))
    
    def set_connected(self, connected):
        """
        Update connection status.
        
        Args:
            connected: True if connected, False otherwise
        """
        if connected != self.is_connected:
            self.is_connected = connected
            print(f"Drone {'connected' if connected else 'disconnected'}")
    
    def get_distance_to(self, x, y, z):
        """
        Calculate distance to a point.
        
        Args:
            x: Target X position
            y: Target Y position
            z: Target Z position
            
        Returns:
            Distance in meters
        """
        return math.sqrt((x - self.x)**2 + (y - self.y)**2 + (z - self.z)**2)
    
    def reset(self):
        """Reset all state information to defaults."""
        self.__init__()
        print("Drone state reset")
    
    def __str__(self):
        """String representation of the drone state."""
        return (
            f"DroneState(pos=({self.x:.2f}, {self.y:.2f}, {self.z:.2f}), "
            f"yaw={self.yaw:.2f}°, state={self.flight_state.name}, "
            f"battery={self.battery_percentage}%, "
            f"connected={self.is_connected})"
        )

# Simple test if run directly
if __name__ == "__main__":
    drone = DroneState()
    print(drone)
    
    # Simulate some updates
    print("\nSimulating takeoff...")
    drone.set_flight_state(FlightState.TAKING_OFF)
    drone.update_position(0, 0, 1.0)
    time.sleep(0.1)
    
    print("\nSimulating flight...")
    drone.set_flight_state(FlightState.FLYING)
    drone.update_position(1.0, 0.5, 1.5)
    drone.update_yaw(45)
    time.sleep(0.1)
    
    print("\nFinal state:")
    print(drone)