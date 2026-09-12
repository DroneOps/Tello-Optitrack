import time
import argparse
import matplotlib.pyplot as plt
from djitellopy import Tello

def run_test(drone, axis, duration):
    vx, vy, vz, y_vel = 0, 0, 0, 0
    movement_desc = ""
    
    speed = 50
    if axis.startswith("-"):
        speed = -50
        base_axis = axis[1:]
    else:
        base_axis = axis

    if base_axis == "x":
        vx = speed # X is Forward
        movement_desc = f"({'+' if speed>0 else '-'}) {'Forward' if speed>0 else 'Backward'}"
    elif base_axis == "y":
        vy = speed # Y is Left
        movement_desc = f"({'+' if speed>0 else '-'}) {'Left' if speed>0 else 'Right'}"
    elif base_axis == "z":
        vz = speed # Z is Up
        movement_desc = f"({'+' if speed>0 else '-'}) {'Up' if speed>0 else 'Down'}"
    elif base_axis == "yaw":
        y_vel = speed
        movement_desc = f"({'+' if speed>0 else '-'}) {'Clockwise' if speed>0 else 'Counter-Clockwise'}"
    
    print(f"Testing {axis.upper()} axis for {duration} seconds... Direction: {movement_desc}")
    data = {"time": [], "vx": [], "vy": [], "vz": [], "height": [], "yaw": []}
    
    start_time = time.time()
    while time.time() - start_time < duration:
        # send_rc_control expects (left_right, forward_backward, up_down, yaw)
        # Note: left_right expects positive for Right. Since our Y is Left, we send -vy.
        drone.send_rc_control(-vy, vx, vz, y_vel)
        
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
    
    import os
    from datetime import datetime
    
    current_dir = os.path.abspath(os.path.dirname(__file__))
    ws_root = current_dir
    while ws_root != "/":
        if os.path.exists(os.path.join(ws_root, "docs")):
            break
        ws_root = os.path.dirname(ws_root)
        
    save_dir = os.path.join(ws_root, 'docs', 'axis_test_plots')
    os.makedirs(save_dir, exist_ok=True)
    
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    file_path = os.path.join(save_dir, f"axis_test_{axis}_{timestamp}.png")
    
    plt.savefig(file_path, bbox_inches='tight')
    
    print("\n" + "="*55)
    print("[+] AXIS TEST PLOT SAVED SUCCESSFULLY!")
    print(f"    -> {file_path}")
    print("="*55 + "\n")
    
    plt.show()

from check_status import connect_and_check

def main():
    parser = argparse.ArgumentParser(description="Test Tello RC commands across different axes.")
    parser.add_argument("--axis", choices=["x", "-x", "y", "-y", "z", "-z", "yaw", "-yaw", "all"], default="z", help="Axis to test (use = for negative, e.g., --axis=-x)")
    parser.add_argument("--duration", type=int, default=3, help="Duration in seconds for each test.")
    args = parser.parse_args()
    
    # Use the unified check status to connect and get telemetry
    drone = connect_and_check()
    
    drone.takeoff()
    time.sleep(2)
    
    axes_to_test = ["x", "-x", "y", "-y", "z", "-z", "yaw", "-yaw"] if args.axis == "all" else [args.axis]
    
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
