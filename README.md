# Tello Drone Control with ROS 2 and OptiTrack

This project implements a **ROS 2** system for the kinematic control of a **DJI Tello** drone using position feedback obtained through an **OptiTrack** motion capture system.

## Overview

The system uses a **Proportional-Integral PI Control** to guide the drone towards a desired point in space. 

**Why PI and not PID?**
In this architecture, the ROS 2 node acts as an outer kinematic control loop. The DJI Tello internal firmware already implements a high-frequency, low-level cascade controller to maintain attitude and velocity. Because the system commands velocity, a first-order kinematic response, rather than acceleration or torque, system inertia is functionally abstracted away. This makes a Derivative D term unnecessary and prone to amplifying motion capture measurement noise. The Integral I term is introduced specifically to eliminate the steady-state error that a purely proportional controller leaves behind due to aerodynamics and battery voltage drops.

## Installation

To run the nodes in this package, you need ROS 2 installed and configured, and some Python libraries. The system was developed and tested on Ubuntu 24.04 with ROS 2, though it should be compatible with other recent ROS 2 distributions.

First, clone this repository. It is essential to use the `--recursive` flag to fetch the `natnet_ros2` submodule:

```bash
git clone --recursive https://github.com/tu_usuario/Tello-Optitrack.git
cd Tello-Optitrack
```

Install the main Python dependencies by running:

```bash
pip install -r requirements.txt
```

The specific Python libraries used are `djitellopy`, `scipy`, `numpy`, and `matplotlib`.

## Hardware Configuration

To ensure the drone movements correspond logically to the motion capture system, configure the physical space and the **Motive** software as follows:

### Axis Alignment
Adjust the ground plane so that the axes align perfectly with the standard ROS coordinate frame (FLU). This prevents any confusion during flight commands:
* **X-axis:** Forward
* **Y-axis:** Left
* **Z-axis:** Up / Altitude

![Ground Plane Adjustment](docs/images/GroundPlane.jpeg)

*Setting the axes in Motive to match the drone inertial frame.*

### OptiTrack Rigid Body
Create a custom Rigid Body inside the Motive software and give it a name. By default, this project expects the name `drone`. This step is crucial because the `natnet_ros2` package automatically maps the Rigid Body name to the ROS 2 topic name. If you use the default name, the system will publish the data over `/drone/pose`. If you use a custom name, you must pass it as a parameter when launching the nodes as described in the How to Run section.

![OptiTrack View 1](docs/images/Optitrack1.jpeg)
![OptiTrack View 2](docs/images/Optitrack2.jpeg)

## Network Configuration

Since the system relies on real-time data streaming from a Windows PC running Motive to a Linux PC running the ROS 2 controller, a solid local network is required.

While you can use any network setup as long as both computers are on the same subnet, **it is highly recommended to connect them directly via an Ethernet cable**. The reason is that the Linux PC needs its WiFi interface free to connect to the Tello drone internal network.

* **Connection:** Connect the Windows PC and Linux PC directly via Ethernet.
* **IP Configuration:** Configure static IPs for the Ethernet ports on both machines. For example:
    * **Windows PC Motive:** `192.168.1.1`
    * **Linux PC ROS 2:** `192.168.1.2`
* **Subnet Warning:** The DJI Tello drone factory WiFi network operates on the `192.168.10.x` subnet. **Do not use the `192.168.10.x` subnet for your Ethernet connection**, otherwise the Linux PC will experience routing conflicts and will not be able to connect to the drone.
* **Streaming Type:** In Motive, configure the OptiTrack data stream as **Unicast**.
* **natnet_ros2 Setup:** Ensure that you properly configure the `natnet_ros2` package settings to match these IPs so it can correctly receive the stream. Please refer to the official [natnet_ros2 README](https://github.com/L2S-lab/natnet_ros2) for detailed instructions on configuring and running their driver.

## How to Run

### 1. Start the OptiTrack Stream
Before running the drone control nodes, you must first launch the OptiTrack driver. Ensure that you have configured it using the IP settings mentioned in the Network Configuration section. 

```bash
ros2 launch natnet_ros2 natnet_ros2.launch.py
```

### 2. Launching the Drone Nodes
Once the OptiTrack data is streaming, you can run the control system and the graphs together using the launch file. By default, the system listens to the `/drone/pose` topic. 

```bash
ros2 launch tello_control controller_with_plot.launch.py
```

**Custom Rigid Body Name:**
If your Rigid Body in Motive is named something else, for example `my_tello`, you can pass it as a parameter so the nodes listen to the correct topic which would be `/my_tello/pose` in this case:

```bash
ros2 launch tello_control controller_with_plot.launch.py rigid_body_name:=my_tello
```

Alternatively, run each node separately. You can also pass the parameter here:

```bash
# Run the control node
ros2 run tello_control tello_controller --ros-args -p rigid_body_name:=my_tello

# Run the pose plotter node
ros2 run tello_control pose_plotter --ros-args -p rigid_body_name:=my_tello
```

### 3. Sending a Target Coordinate
Once the system is running, the `tello_controller` node will stay in a waiting state until a goal is published. 

You can use the `send_goal.sh` script located in the `utils` folder to simplify this process:

```bash
./utils/send_goal.sh -1.0 1.0 1.0
```

### 4. Running the Square Routine
We have provided an automated square routine that flies the drone through four corners and returns to the center. It uses feedback from the controller to ensure it has reached a point before moving to the next.

To run it, simply execute:
```bash
ros2 run tello_control square_routine
```
When it finishes, the drone will hover at the center `(0, 0, 1)`. You can land the drone at any time by pressing `Ctrl+C` in the terminal running the `tello_controller` node.

## Utilities

The `utils` folder contains standalone scripts for hardware testing and automation:

* **axis_test.py**: Standalone script to test the drone's physical response and sensors without using OptiTrack. It connects to the drone, takes off, sends a pure RC velocity command on a specified axis, logs the internal IMU telemetry, lands, and generates a `.png` plot with the results.
  Run it from the root of the workspace:
  ```bash
  python3 utils/axis_test.py --axis=all --duration=3
  ```
  **Arguments:**
  * `--axis`: The axis to test. Available options are `x`, `-x`, `y`, `-y`, `z`, `-z`, `yaw`, `-yaw`, or `all`. If no arguments are passed, the drone will safely default to `z` and move Up.
  * `--duration`: Time in seconds to hold the velocity command per axis. Default is `3`.

* **connect_wifi.sh**: A bash script that reads the SSID and password from the `tello.conf` configuration file and forces the Linux network manager to connect to the Tello network.

* **check_status.py**: A Python script that loops the WiFi connection process until successful, initializes the DJI SDK, and prints out the drone battery level and internal temperature. This module is used internally by the main controller to guarantee the drone is ready before taking off.

## Software Architecture

The project is divided into key packages and nodes that communicate via a Publisher-Subscriber architecture:

1. **OptiTrack Package `natnet_ros2`**: 
   * **Publishes:** `/drone/pose` geometry_msgs/PoseStamped.
   * Fetches tracking data from the Motive server and sends it to the Linux PC.
   * *Note: The `natnet_ros2` package (included as a submodule) is developed by L2S-lab at https://github.com/L2S-lab/natnet_ros2. Citation for their work can be found at https://hal.science/hal-04150950.*

2. **Control Node `tello_controller`**: 
   * **Subscribes to:** `/drone/pose` to get the current drone position and `/goal` to get the desired target.
   * Calculates the proportional control velocities and directly sends RC commands to the Tello drone over WiFi via the `djitellopy` library.

3. **Pose Plotter `pose_plotter`**: 
   * **Subscribes to:** `/drone/pose` and `/goal`.
   * Plots the trajectory and behavior of the drone in real-time, allowing visual evaluation of the controller performance.

## Control Theory and Mathematics

### Kinematic Error and Discrete PI Control Law
Since the controller runs on a digital computer, the control law is implemented numerically in discrete time steps ($k$). The discrete position error $e[k]$ is calculated in the global inertial frame as:

$$e[k] = P_{target} - P_{current}[k]$$

A Proportional-Integral (PI) control law is applied to compute the required velocity. $K_p$ is the proportional gain matrix, $K_i$ is the integral gain matrix, and $\Delta t$ represents the time elapsed between sensor callbacks. The numerical approximation of the integral is handled via an error accumulator:

$$V_{inertial}[k] = K_p \cdot e[k] + K_i \sum_{j=0}^{k} e[j] \Delta t$$

### Inverse Rotation Matrix
Orientation data from OptiTrack is received as quaternions $(q_x, q_y, q_z, q_w)$. This is mathematically converted to Euler angles to extract the drone Yaw angle ($\psi$). 

Since the Tello expects movement commands relative to its own local body frame rather than the global frame, the inertial velocity must be transformed. A 2D inverse rotation matrix around the Z-axis is applied to map global vectors into the drone intrinsic axes:

$$V_{body}[k] = R_z^{-1}(\psi) V_{inertial}[k]$$

Where the inverse rotation matrix $R_z^{-1}(\psi) = R_z(-\psi)$ is defined as:
$$R_z(-\psi) = \begin{bmatrix} \cos(\psi) & \sin(\psi) & 0 \\ -\sin(\psi) & \cos(\psi) & 0 \\ 0 & 0 & 1 \end{bmatrix}$$

### RC Command Mapping
The Tello SDK does not accept direct metric velocity commands ($m/s$). Instead, the computed $V_{body}$ vector is scaled by a constant factor and saturated (clamped) to the $[-100, 100]$ integer range. These normalized values are then transmitted via the `send_rc_control` function, translating the theoretical velocities into standard RC channel inputs (roll, pitch, throttle).

## Flight Results

The plot below illustrates the final pose during a flight test. The trajectory demonstrates the drone converging towards the target position. By implementing the Integral term in the PI controller, the steady-state error is eliminated, allowing the system to reach the exact reference coordinates.

![Final Pose](docs/images/FinalPose.jpeg)

The following graphs detail the individual spatial coordinates and the drone Yaw angle over time. The response shows a smooth approach to the desired setpoint, confirming the stability and zero steady-state error achieved by the Proportional-Integral control architecture.

![Final Graphs](docs/images/FInalGraphs.jpeg)

