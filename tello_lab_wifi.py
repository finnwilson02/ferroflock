import socket
import time

def get_socket():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(('', 8889))
    s.settimeout(5)
    return s

def force_close_connection():
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(('', 8889))
        sock.settimeout(2)
        sock.sendto(b'emergency', ('192.168.10.1', 8889))
        time.sleep(1)
        sock.sendto(b'motoroff', ('192.168.10.1', 8889))
    except:
        pass
    finally:
        sock.close()
        time.sleep(2)

def set_ap_mode(ssid, password, address=('192.168.10.1', 8889)):
    s = get_socket()

    # Enter SDK mode
    cmd = 'command'
    print(f'Sending command: {cmd}')
    s.sendto(cmd.encode('utf-8'), address)
    try:
        response, ip = s.recvfrom(100)
        response_text = response.decode()
        print(f'Response from {ip}: {response_text}')
        if response_text.lower() != 'ok':
            return False, "Failed to enter SDK mode"
    except socket.timeout:
        return False, "No response to command mode"

    # Set AP mode
    cmd = f'ap {ssid} {password}'
    print(f'Sending command: {cmd}')
    s.sendto(cmd.encode('utf-8'), address)
    try:
        response, ip = s.recvfrom(100)
        response_text = response.decode()
        print(f'Response from {ip}: {response_text}')
        
        # Check for successful reboot message
        if "reboot" in response_text.lower():
            return True, "Drone will reboot and connect to WiFi"
        return False, "Unexpected response"
    except socket.timeout:
        return False, "No response to AP mode command"

def main():
    # Lab WiFi credentials
    LAB_SSID = "UAV-LAB-2.4G"
    LAB_PASSWORD = "idontknow"

    print("Cleaning up any existing connections...")
    force_close_connection()

    print("\nSetting AP mode and connecting to lab WiFi...")
    success, message = set_ap_mode(LAB_SSID, LAB_PASSWORD)

    if success:
        print("\n✅ Configuration successful!")
        print(f"Status: {message}")
        print("\nWhat's happening now:")
        print("1. The drone will reboot automatically")
        print("2. The LED will change colors during the process")
        print("3. The drone will connect to the lab WiFi network")
        print("\nNext steps:")
        print("1. Wait about 30 seconds for the reboot to complete")
        print("2. Use this command to find the drone's new IP:")
        print("   sudo nmap -sn 192.168.1.0/24")
        print("3. Look for MAC addresses starting with 34:D2:62: (DJI)")
    else:
        print("\n❌ Configuration failed!")
        print(f"Error: {message}")
        print("\nTroubleshooting tips:")
        print("1. If LED is purple: Power cycle the drone")
        print("2. If LED is not blinking yellow: Hold power button for 5 seconds to reset")
        print("3. Make sure you're connected to the Tello's WiFi network")

if __name__ == "__main__":
    main()