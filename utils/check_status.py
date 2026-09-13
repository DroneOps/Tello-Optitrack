"""
Pre-flight check utility.
Connects to Tello WiFi, initializes the SDK, and verifies battery and temperature.
"""

import time
import subprocess
import os
import logging
from djitellopy import Tello

# Suppress djitellopy logs
Tello.LOGGER.setLevel(logging.ERROR)


def connect_and_check():
    """Connect to Tello WiFi, verify telemetry, and return drone instance."""
    script_dir = os.path.dirname(os.path.abspath(__file__))
    wifi_script = os.path.join(script_dir, "connect_wifi.sh")

    print("\n" + "=" * 55)
    print("               TELLO PRE-FLIGHT SYSTEM               ")
    print("=" * 55)
    print("[*] SCANNING: Waiting for Tello WiFi network...")

    while True:
        try:
            result = subprocess.run([wifi_script], capture_output=True, text=True)
            if result.returncode == 0:
                print("[+] NETWORK: WiFi connection established successfully.")
                break
            else:
                print("[-] STANDBY: WiFi not ready. Retrying in 3 seconds...")
                time.sleep(3)
        except KeyboardInterrupt:
            print("\n[!] ABORT: Connection aborted by user.")
            exit(1)

    print("[*] HANDSHAKE: Connecting to DJI Tello SDK...")
    drone = Tello()
    drone.connect()

    battery = drone.get_battery()
    temp = drone.get_temperature()

    print("-" * 55)
    print("                   TELEMETRY DATA                    ")
    print("-" * 55)
    print(f"    BATTERY LEVEL        : {battery}%")
    print(f"    INTERNAL TEMPERATURE : {temp} C")
    print("-" * 55)

    if int(temp) >= 85:
        print(" [!] CRITICAL: DRONE IS TOO HOT TO FLY! (>85C)       ")
        print(" [!] The Tello firmware will block the motors.       ")
        print(" [!] Please turn it off and let it cool down.        ")
        print("=" * 55 + "\n")
        import sys

        sys.exit(1)

    if int(battery) < 10:
        print(" [!] CRITICAL: BATTERY TOO LOW TO FLY! (<10%)        ")
        print("=" * 55 + "\n")
        import sys

        sys.exit(1)

    print("              SYSTEM READY FOR TAKEOFF               ")
    print("=" * 55 + "\n")

    return drone


def main():
    drone = connect_and_check()
    drone.end()


if __name__ == "__main__":
    main()
