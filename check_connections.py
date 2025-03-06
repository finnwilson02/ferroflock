from djitellopy import Tello
import time

def check_tello_connection():
    tello = Tello()
    
    try:
        print("Attempting to connect to Tello...")
        tello.connect()
        time.sleep(2)  # Give it time to establish connection
        
        # Check basic stats
        battery = tello.get_battery()
        temp = tello.get_temperature()
        height = tello.get_height()
        
        print("\nConnection Status:")
        print(f"✓ Successfully connected to Tello")
        print(f"✓ Battery Level: {battery}%")
        print(f"✓ Temperature: {temp}°C")
        print(f"✓ Height: {height}cm")
        
        try:
            wifi_snr = tello.query_wifi_signal_noise_ratio()
            print(f"✓ WiFi Signal-to-Noise Ratio: {wifi_snr}")
        except:
            print("× Could not get WiFi signal strength")
            
    except Exception as e:
        print(f"\n× Connection failed: {str(e)}")
        
    finally:
        try:
            tello.end()
        except:
            pass

if __name__ == "__main__":
    check_tello_connection()