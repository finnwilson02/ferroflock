import subprocess
import re
import json
import os
from datetime import datetime

def find_dji_devices():
    # Run nmap to find all devices
    result = subprocess.run(['sudo', 'nmap', '-sn', '192.168.1.0/24'], 
                          capture_output=True, text=True)
    
    # Parse nmap output for DJI devices
    current_devices = []
    current_ip = None
    
    for line in result.stdout.split('\n'):
        ip_match = re.search(r'Nmap scan report for (192\.168\.1\.\d+)', line)
        mac_match = re.search(r'MAC Address: ([\w:]+) \(([^)]+)\)', line)
        
        if ip_match:
            current_ip = ip_match.group(1)
        elif mac_match and current_ip:
            mac = mac_match.group(1)
            manufacturer = mac_match.group(2)
            
            # Check if it's a DJI device (34:D2:62)
            if mac.upper().startswith('34:D2:62'):
                current_devices.append({
                    'ip': current_ip,
                    'mac': mac.upper(),
                    'manufacturer': manufacturer,
                    'last_seen': datetime.now().isoformat(),
                    'online': True  # This drone is currently online
                })
    
    # Load existing device data if available
    output_path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'dji_devices.json')
    existing_devices = []
    try:
        if os.path.exists(output_path):
            with open(output_path, 'r') as f:
                existing_devices = json.load(f)
                print(f"Loaded {len(existing_devices)} devices from existing configuration")
    except Exception as e:
        print(f"Error loading existing device data: {e}")
    
    # Create a map of MAC addresses to devices for easier lookup
    mac_to_device = {dev['mac']: dev for dev in existing_devices}
    
    # Prepare final device list:
    # 1. Include all currently detected devices (marked as online)
    # 2. Include previously seen devices not currently detected (marked as offline)
    final_devices = []
    
    # Add current devices
    for device in current_devices:
        # If we've seen this device before, preserve its calibration data
        if device['mac'] in mac_to_device:
            old_device = mac_to_device[device['mac']]
            
            # Keep any existing fields from the old device record
            for key in old_device:
                if key not in device and key != 'online' and key != 'last_seen':
                    device[key] = old_device[key]
            
            # Remove this MAC from the map so we know it's been processed
            del mac_to_device[device['mac']]
        
        final_devices.append(device)
    
    # Add offline devices (previously seen but not currently detected)
    for mac, device in mac_to_device.items():
        device['online'] = False
        final_devices.append(device)
    
    # Remove offline devices if they're causing issues
    final_devices = [d for d in final_devices if d.get('online', False)]
    
    # Save to JSON file using absolute path in parent directory
    with open(output_path, 'w') as f:
        json.dump(final_devices, f, indent=2)
    
    print(f"Saved device information to {output_path}")
    
    # Count online devices
    online_count = sum(1 for dev in final_devices if dev.get('online', False))
    print(f"Found {online_count} online DJI devices - saved ONLY online devices")
    
    return final_devices

if __name__ == "__main__":
    devices = find_dji_devices()
    
    online_devices = [d for d in devices if d.get('online', False)]
    offline_devices = [d for d in devices if not d.get('online', False)]
    
    print(f"\nCurrent Online DJI Devices ({len(online_devices)}):")
    for device in online_devices:
        print(f"IP: {device['ip']}, MAC: {device['mac']}")
    
    if offline_devices:
        print(f"\nOffline DJI Devices with Saved Configuration ({len(offline_devices)}):")
        for device in offline_devices:
            print(f"MAC: {device['mac']}, Last IP: {device.get('ip', 'unknown')}, Last seen: {device.get('last_seen', 'unknown')}")