import socket
import json
import time
import subprocess
import numpy as np

class DroneCalibrator:
    def __init__(self):
        # Connect to VRPN data server
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.connect(('127.0.0.1', 8080))
        
        # Load device information
        with open('dji_devices.json', 'r') as f:
            self.devices = json.load(f)
        
        self.mapping = {}
        
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
    
    def identify_drone(self, ip):
        print(f"\nTesting drone at IP: {ip}")
        
        # Get baseline positions
        base_data = self.get_tracker_data()
        base_positions = {k: np.array(v['pos']) for k, v in base_data.items()}
        
        # Make drone move
        subprocess.run(['./swarm', '--single', ip, '--command', 'takeoff'])
        time.sleep(2)
        
        # Get new positions
        test_data = self.get_tracker_data()
        test_positions = {k: np.array(v['pos']) for k, v in test_data.items()}
        
        # Land drone
        subprocess.run(['./swarm', '--single', ip, '--command', 'land'])
        
        # Find which tracker moved the most
        max_movement = 0
        moved_tracker = None
        
        for name in base_positions:
            if name in test_positions:
                movement = np.linalg.norm(test_positions[name] - base_positions[name])
                if movement > max_movement:
                    max_movement = movement
                    moved_tracker = name
        
        return moved_tracker if max_movement > 0.1 else None
    
    def map_all_drones(self):
        for device in self.devices:
            print(f"\nMapping drone: {device['mac']}")
            print("Please ensure space is clear around this drone")
            
            # Flash LED to identify drone
            subprocess.run(['./swarm', '--single', device['ip'], 
                          '--command', 'led 255 0 0'])
            input("Press Enter when ready...")
            
            tracker_id = self.identify_drone(device['ip'])
            if tracker_id:
                print(f"Mapped to tracker: {tracker_id}")
                self.mapping[device['mac']] = {
                    'ip': device['ip'],
                    'tracker_id': tracker_id
                }
            else:
                print("Failed to identify tracker!")
            
            # Turn off LED
            subprocess.run(['./swarm', '--single', device['ip'], 
                          '--command', 'led 0 0 0'])
        
        # Save mapping
        with open('drone_mapping.json', 'w') as f:
            json.dump(self.mapping, f, indent=2)

if __name__ == "__main__":
    calibrator = DroneCalibrator()
    calibrator.map_all_drones()