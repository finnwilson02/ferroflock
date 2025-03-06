#!/usr/bin/env python3
# get_devices.py - Utility to scan the network for Tello drones

import sys
import os
import json
import socket
import time
import argparse
from datetime import datetime

def get_devices(output_dir):
    """
    Scan the network for Tello drones and save the results to a JSON file.
    
    Args:
        output_dir: Directory where to save the JSON file
    
    Returns:
        dict: Dictionary containing the discovered devices
    """
    # Create the output directory if it doesn't exist
    os.makedirs(output_dir, exist_ok=True)
    
    # Prepare a socket for broadcasting
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    sock.settimeout(5.0)  # 5 seconds timeout
    
    # IP ranges to scan (Tello drones usually use 192.168.10.1 or 192.168.10.2)
    ip_ranges = [
        "192.168.1.255",  # For birds network
        "192.168.10.255"  # For normal Tello network
    ]
    
    # Command to send
    command = b'command'
    
    # Dictionary to store the discovered devices
    devices = []
    
    print("Scanning for Tello drones...")
    
    # Try to bind to port 8889 (Tello command port)
    try:
        sock.bind(('', 8889))
    except socket.error:
        print("Warning: Couldn't bind to port 8889. Are you running as root?")
        try:
            # Try another port
            sock.bind(('', 0))
        except socket.error as e:
            print(f"Error: Couldn't bind to any port: {e}")
            return {}
    
    # Send broadcast commands to every IP range
    for ip in ip_ranges:
        print(f"Broadcasting to {ip}...")
        sock.sendto(command, (ip, 8889))
    
    # List of known IPs to avoid duplicates
    known_ips = set()
    
    # Wait for responses for 10 seconds
    end_time = time.time() + 10
    while time.time() < end_time:
        try:
            # Receive response
            response, addr = sock.recvfrom(1024)
            
            ip, port = addr
            
            # Skip if we've already seen this IP
            if ip in known_ips:
                continue
            
            known_ips.add(ip)
            
            print(f"Found drone at {ip}: {response.decode('utf-8').strip()}")
            
            # Add to our devices list with appropriate metadata
            device = {
                "ip": ip,
                "port": 8889,
                "name": f"Tello-{len(devices)+1}",
                "type": "tello",
                "model": "Tello EDU",
                "discovered": datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            }
            
            # Special mappings for specific IPs
            # These are the mappings from your existing code
            tracker_mapping = {
                "192.168.1.107": "Bird5",
                "192.168.1.106": "Bird3",
                "192.168.1.104": "Bird4",
                "192.168.1.103": "Bird1"
            }
            
            if ip in tracker_mapping:
                device["name"] = tracker_mapping[ip]
                device["tracker_id"] = tracker_mapping[ip]
            else:
                device["tracker_id"] = f"Tracker{len(devices)+1}"
            
            devices.append(device)
            
        except socket.timeout:
            # No more responses
            break
    
    # Close the socket
    sock.close()
    
    # Create the output JSON file
    output_file = os.path.join(output_dir, "dji_devices.json")
    
    with open(output_file, 'w') as f:
        json.dump({"devices": devices}, f, indent=2)
    
    print(f"Found {len(devices)} devices. Results saved to {output_file}")
    
    return {"devices": devices}

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Scan the network for Tello drones")
    parser.add_argument("--output-dir", default="drone_info", help="Directory where to save the JSON file")
    args = parser.parse_args()
    
    get_devices(args.output_dir)