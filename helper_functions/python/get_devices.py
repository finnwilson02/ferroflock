#!/usr/bin/env python3
# get_devices.py - Utility to scan the network for Tello drones
# Requires scapy: Run `pip install scapy` in your virtual environment
'''
Purpose: Discovers DJI Tello drones on the network using ARP scanning

Data Flow:
  Input: ARP scan results of the 192.168.1.0/24 subnet
  Output: JSON file (~/ferroflock/dji_devices.json) containing device information

This script scans for Tello drones using their MAC address prefix (34:d2:62),
maintains device status (online/offline), and handles MAC/IP conflicts.
'''

import sys
import os
import json
import argparse
from datetime import datetime
from scapy.all import ARP, Ether, srp

def load_existing_devices(json_path):
    """Load existing devices from JSON file if it exists."""
    if os.path.exists(json_path):
        try:
            with open(json_path, 'r') as f:
                data = json.load(f)
                return data.get("devices", [])
        except json.JSONDecodeError:
            print(f"Warning: Error parsing {json_path}, treating as empty")
            return []
        except Exception as e:
            print(f"Warning: Error reading {json_path}: {e}")
            return []
    return []

def get_devices(output_dir):
    """
    Scan the network for Tello drones, update existing JSON, and save to ~/ferroflock/dji_devices.json.
    
    Args:
        output_dir: Ignored - output is forced to ~/ferroflock/dji_devices.json
    
    Returns:
        dict: Dictionary containing the discovered devices
    """
    # Force output to base directory
    base_dir = os.path.expanduser("~/ferroflock")
    output_file = os.path.join(base_dir, "dji_devices.json")

    # Load existing devices
    existing_devices = load_existing_devices(output_file)
    ip_to_device = {d["ip"]: d for d in existing_devices}
    mac_to_device = {d.get("mac", "").lower(): d for d in existing_devices if "mac" in d}

    # Define subnet to scan
    subnet = "192.168.1.0/24"
    print(f"Scanning {subnet} for Tello drones...")

    # Create ARP request packet
    arp = ARP(pdst=subnet)
    ether = Ether(dst="ff:ff:ff:ff:ff:ff")
    packet = ether / arp

    # Send packet and receive responses
    answered, _ = srp(packet, timeout=5, verbose=False, iface="wlp3s0")

    # Debug: Print scapy results
    print(f"DEBUG: Total packets answered: {len(answered)}")
    for sent, received in answered:
        print(f"DEBUG: Detected IP: {received.psrc}, MAC: {received.hwsrc.lower()}")

    # Track discovered devices
    devices = existing_devices.copy()  # Start with existing devices
    known_ips = set(d["ip"] for d in devices)
    known_macs = set(d.get("mac", "").lower() for d in devices if "mac" in d)
    dji_mac_prefix = "34:d2:62"
    
    # Mark all devices as offline initially
    for device in devices:
        device["status"] = "offline"

    # Process scan results
    for sent, received in answered:
        ip = received.psrc
        mac = received.hwsrc.lower()

        if not mac.startswith(dji_mac_prefix.lower()):
            continue  # Skip non-DJI devices

        print(f"Detected drone at {ip} (MAC: {mac})")

        # Check for MAC/IP conflicts
        if mac in known_macs:
            old_device = mac_to_device[mac]
            old_ip = old_device["ip"]
            
            if ip != old_ip:
                print(f"Conflict: MAC {mac} has IP {ip} but was previously {old_ip}")
                choice = input("Overwrite old entry with new IP? (y/n): ").strip().lower()
                
                if choice == "y":
                    # Update IP, preserve other fields
                    old_device["ip"] = ip
                    old_device["status"] = "online"
                    old_device["discovered"] = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                    
                    # If old IP is reused with different MAC, mark it
                    if ip in ip_to_device and ip_to_device[ip]["mac"].lower() != mac:
                        ip_to_device[ip]["status"] = "IP REFRESH NEEDED"
                else:
                    # Mark the device with conflict status
                    old_device["status"] = "IP CONFLICT"
                    
                # Skip adding new entry since we updated the existing one
                continue
            else:
                # MAC matches existing entry with same IP, just mark as online
                old_device["status"] = "online"
                old_device["discovered"] = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                continue

        # Check for IP conflicts (different MAC, same IP)
        if ip in ip_to_device:
            existing_device = ip_to_device[ip]
            existing_mac = existing_device.get("mac", "").lower()
            
            if existing_mac and existing_mac != mac:
                print(f"Conflict: IP {ip} has MAC {mac} but was previously {existing_mac}")
                choice = input("Overwrite existing entry? (y/n): ").strip().lower()
                
                if choice == "y":
                    # Update MAC, preserve other fields
                    existing_device["mac"] = mac
                    existing_device["status"] = "online"
                    existing_device["discovered"] = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                else:
                    # Mark the device with conflict status
                    existing_device["status"] = "MAC CONFLICT"
                
                continue
            else:
                # Update existing device
                existing_device["mac"] = mac
                existing_device["status"] = "online" 
                existing_device["discovered"] = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                continue

        # Add new device
        device = {
            "ip": ip,
            "port": 8889,
            "name": f"Tello-{len(devices)+1}",
            "type": "tello",
            "model": "Tello EDU",
            "mac": mac,
            "status": "online",
            "discovered": datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        }
        
        # Special mappings for specific IPs
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
            device["tracker_id"] = f"Tracker{len([d for d in devices if 'tracker_id' in d])+1}"
        
        devices.append(device)
        ip_to_device[ip] = device
        known_macs.add(mac)
        known_ips.add(ip)

    # Debug: Confirm online devices
    online_count = len([d for d in devices if d.get("status") == "online"])
    print(f"DEBUG: Devices marked online after scan: {online_count}")

    # Save updated devices
    with open(output_file, 'w') as f:
        json.dump({"devices": devices}, f, indent=2)

    print(f"Updated {len(devices)} devices. Results saved to {output_file}")
    print(f"Online devices: {len([d for d in devices if d.get('status') == 'online'])}")
    print(f"Offline devices: {len([d for d in devices if d.get('status') == 'offline'])}")
    return {"devices": devices}

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Scan the network for Tello drones")
    parser.add_argument("--output-dir", default="ignored", help="Ignored - saves to ~/ferroflock/dji_devices.json")
    args = parser.parse_args()
    
    get_devices(args.output_dir)