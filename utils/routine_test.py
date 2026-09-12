import time
import argparse
import matplotlib.pyplot as plt
from djitellopy import Tello

def run_test(drone, axis, duration):
    vx, vy, vz, yaw = 0, 0, 0, 0
    
    if axis == "x":
        vx = 50
    elif axis == "y":
        vy = 50
    elif axis == "z":
        vz = 50
    elif axis == "yaw":
        yaw = 50
    
    print(f"Testing {axis.upper()} axis for {duration} seconds...")
    data = {"time": [], "vx": [], "vy": [], "vz": [], "height": [], "yaw": []}
    
    start_time = time.time()
    while time.time() - start_time < duration:
        drone.send_rc_control(vx, vy, vz, yaw)
        
        data["time"].append(time.time() - start_time)
        data["vx"].append(drone.get_speed_x())
        data["vy"].append(drone.get_speed_y())
        data["vz"].append(drone.get_speed_z())
        data["height"].append(drone.get_height())
        data["yaw"].append(drone.get_yaw())
        
        time.sleep(0.1)
        
    drone.send_rc_control(0, 0, 0, 0)
    time.sleep(1)
    return data

def plot_data(data, axis):
    fig, axs = plt.subplots(3, 1, figsize=(10, 10))
    fig.suptitle(f"Tello Telemetry - Test: {axis.upper()}")
    
    axs[0].plot(data["time"], data["vx"], label="Vx", color='r')
    axs[0].plot(data["time"], data["vy"], label="Vy", color='g')
    axs[0].plot(data["time"], data["vz"], label="Vz", color='b')
    axs[0].set_ylabel("Speed")
    axs[0].legend()
    axs[0].grid(True)
    
    axs[1].plot(data["time"], data["height"], label="Height", color='m')
    axs[1].set_ylabel("Height cm")
    axs[1].legend()
    axs[1].grid(True)
    
    axs[2].plot(data["time"], data["yaw"], label="Yaw", color='c')
    axs[2].set_ylabel("Yaw degrees")
    axs[2].set_xlabel("Time s")
    axs[2].legend()
    axs[2].grid(True)
    
    plt.tight_layout()
    plt.savefig(f"plot_{axis}.png")
    print(f"Plot saved as plot_{axis}.png")
    plt.show()

from check_status import connect_and_check

def main():
    parser = argparse.ArgumentParser(description="Test Tello RC commands across different axes.")
    parser.add_argument("--axis", choices=["x", "y", "z", "yaw", "all"], default="z", help="Axis to test.")
    parser.add_argument("--duration", type=int, default=3, help="Duration in seconds for each test.")
    args = parser.parse_args()
    
    # Use the unified check status to connect and get telemetry
    drone = connect_and_check()
    
    drone.takeoff()
    time.sleep(2)
    
    axes_to_test = ["x", "y", "z", "yaw"] if args.axis == "all" else [args.axis]
    
    all_data = {"time": [], "vx": [], "vy": [], "vz": [], "height": [], "yaw": []}
    global_time_offset = 0
    
    for ax in axes_to_test:
        run_data = run_test(drone, ax, args.duration)
        
        for i in range(len(run_data["time"])):
            all_data["time"].append(run_data["time"][i] + global_time_offset)
            all_data["vx"].append(run_data["vx"][i])
            all_data["vy"].append(run_data["vy"][i])
            all_data["vz"].append(run_data["vz"][i])
            all_data["height"].append(run_data["height"][i])
            all_data["yaw"].append(run_data["yaw"][i])
            
        global_time_offset = all_data["time"][-1] if all_data["time"] else 0
        
    drone.land()
    drone.end()
    
    plot_data(all_data, args.axis)

if __name__ == "__main__":
    main()
