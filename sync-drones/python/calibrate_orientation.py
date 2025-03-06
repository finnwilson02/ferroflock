import socket
import json
import time
import subprocess
import numpy as np
from scipy.spatial.transform import Rotation

class DroneOrientationCalibrator:
    def __init__(self):
        # Connect to VRPN data server
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.connect(('127.0.0.1', 8080))
        
        # Load mapping from previous calibration
        with open('drone_mapping.json', 'r') as f:
            self.mapping = json.load(f)
            
        self.orientation_corrections = {}
        
    def get_tracker_data(self):
        data = self.socket.recv(1024).decode('utf-8')
        trackers = {}
        for line in data.strip().split('\n'):
            parts = line.split(',')
            if len(parts) == 8:  # name, x, y, z, qw, qx, qy, qz
                trackers[parts[0]] = {
                    'pos': [float(parts[1]), float(parts[2]), float(parts[3])],
                    'rot': [float(parts[4]), float(parts[5]), 
                           float(parts[6]), float(parts[7])]
                }
        return trackers

    def calculate_correction(self, current_quat, reference_quat=None):
        """Calculate correction quaternion to align with reference"""
        if reference_quat is None:
            # If no reference provided, use identity (facing forward)
            reference_quat = [1, 0, 0, 0]
            
        # Convert to scipy rotations
        current_rot = Rotation.from_quat([current_quat[1], current_quat[2], 
                                        current_quat[3], current_quat[0]])  # xyzw to wxyz
        ref_rot = Rotation.from_quat([reference_quat[1], reference_quat[2], 
                                    reference_quat[3], reference_quat[0]])
        
        # Calculate correction (inverse of current * reference)
        correction = ref_rot * current_rot.inv()
        
        # Convert back to quaternion (wxyz)
        quat = correction.as_quat()
        return [quat[3], quat[0], quat[1], quat[2]]  # Convert back to wxyz

    def calibrate_orientation(self, mac_address):
        info = self.mapping[mac_address]
        tracker_id = info['tracker_id']
        
        # Flash LED sequence to identify drone
        print(f"\nCalibrating drone: {mac_address}")
        print(f"Looking for tracker: {tracker_id}")
        subprocess.run(['./swarm', '--single', info['ip'], 
                      '--command', 'led 255 0 0'])
        
        print("Place drone on calibration surface facing forward")
        print("Forward direction should be along the X axis (red) in OptiTrack")
        input("Press Enter when drone is positioned correctly...")

        # Get a few samples of orientation to average
        orientations = []
        for _ in range(10):
            tracker_data = self.get_tracker_data()
            if tracker_id in tracker_data:
                orientations.append(tracker_data[tracker_id]['rot'])
            time.sleep(0.1)
        
        if not orientations:
            print(f"Failed to get orientation data for {tracker_id}")
            return False
            
        # Average quaternions (simple average for small variations)
        avg_quat = np.mean(orientations, axis=0)
        avg_quat = avg_quat / np.linalg.norm(avg_quat)  # Normalize
        
        # Calculate correction
        correction = self.calculate_correction(avg_quat)
        
        # Store correction
        self.orientation_corrections[mac_address] = {
            'correction_quaternion': correction,
            'reference_euler': self.quat_to_euler(correction)
        }
        
        # Turn off LED
        subprocess.run(['./swarm', '--single', info['ip'], 
                      '--command', 'led 0 0 0'])
        
        print(f"Calibrated orientation correction for {mac_address}")
        print(f"Euler angles (degrees): {self.orientation_corrections[mac_address]['reference_euler']}")
        return True

    @staticmethod
    def quat_to_euler(quat):
        """Convert quaternion to euler angles in degrees"""
        rot = Rotation.from_quat([quat[1], quat[2], quat[3], quat[0]])  # xyzw to wxyz
        euler = rot.as_euler('xyz', degrees=True)
        return euler.tolist()

    def verify_orientation(self, mac_address):
        """Verify correction by showing live corrected orientation"""
        info = self.mapping[mac_address]
        tracker_id = info['tracker_id']
        correction = self.orientation_corrections[mac_address]['correction_quaternion']
        
        print(f"\nVerifying orientation for {mac_address}")
        print("Rotate drone to verify correction")
        print("Press Ctrl+C to finish verification")
        
        try:
            while True:
                tracker_data = self.get_tracker_data()
                if tracker_id in tracker_data:
                    current = tracker_data[tracker_id]['rot']
                    # Apply correction
                    corrected_rot = Rotation.from_quat([current[1], current[2], 
                                                      current[3], current[0]]) * \
                                  Rotation.from_quat([correction[1], correction[2], 
                                                    correction[3], correction[0]])
                    euler = corrected_rot.as_euler('xyz', degrees=True)
                    print(f"\rCorrected Euler angles: Roll={euler[0]:.1f}° "
                          f"Pitch={euler[1]:.1f}° Yaw={euler[2]:.1f}°", end='')
                time.sleep(0.1)
        except KeyboardInterrupt:
            print("\nVerification complete")

    def calibrate_all_drones(self):
        print("Starting orientation calibration...")
        print("Please ensure you have a flat calibration surface")
        print("Orient it along the X axis in OptiTrack")
        
        for mac in self.mapping:
            if self.calibrate_orientation(mac):
                verify = input("Would you like to verify this calibration? (y/n): ")
                if verify.lower() == 'y':
                    self.verify_orientation(mac)
        
        # Save calibrations
        full_calibration = {
            'mapping': self.mapping,
            'orientations': self.orientation_corrections
        }
        with open('drone_calibrations.json', 'w') as f:
            json.dump(full_calibration, f, indent=2)

if __name__ == "__main__":
    calibrator = DroneOrientationCalibrator()
    calibrator.calibrate_all_drones()