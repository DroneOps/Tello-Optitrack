import time
import subprocess
import os
from djitellopy import Tello

def connect_and_check():
    """
    Loop until the drone WiFi is found and connected.
    Then connect the SDK and print basic status.
    Returns the connected Tello object.
    """
    script_dir = os.path.dirname(os.path.abspath(__file__))
    wifi_script = os.path.join(script_dir, "connect_wifi.sh")
    
    print("Waiting for Tello WiFi network to be available...")
    while True:
        try:
            result = subprocess.run([wifi_script], capture_output=True, text=True)
            if result.returncode == 0:
                print("WiFi connection established successfully.")
                break
            else:
                print("WiFi not ready. Retrying in 3 seconds. Press Ctrl+C to abort.")
                time.sleep(3)
        except KeyboardInterrupt:
            print("\nConnection aborted by user.")
            exit(1)
            
    print("Connecting to Tello SDK...")
    drone = Tello()
    drone.connect()
    
    battery = drone.get_battery()
    temp = drone.get_temperature()
    
    print(f"Battery Level: {battery}%")
    print(f"Internal Temperature: {temp} C")
    
    return drone

def main():
    drone = connect_and_check()
    drone.end()

if __name__ == '__main__':
    main()
