import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from djitellopy import Tello
from scipy.spatial.transform import Rotation as R
import numpy as np
import signal
import sys
import os

"""
Kinematic PI position controller for DJI Tello.
Receives OptiTrack feedback and tracks 3D setpoints from /goal.
"""


class TelloController(Node):
    def __init__(self):
        super().__init__("tello_controller")

        # Rigid body parameter
        self.declare_parameter("rigid_body_name", "drone")
        self.rigid_body_name = (
            self.get_parameter(
                "rigid_body_name").get_parameter_value().string_value
        )

        # Flight state flag
        self.has_taken_off = False

        # OptiTrack pose subscriber
        optitrack_topic = f"/{self.rigid_body_name}/pose"
        self.subscription = self.create_subscription(
            PoseStamped,
            optitrack_topic,
            self.data_callback,
            10,
        )
        # Goal subscriber
        self.goal_sub = self.create_subscription(
            PoseStamped, "/goal", self.goal_callback, 10
        )

        # Goal reached publisher
        self.reached_pub = self.create_publisher(Bool, "/goal_reached", 10)

        # Connect to drone
 

        # Search for the utils folder by walking up the directory tree
        # This works regardless of symlink-install or regular install
        current_dir = os.path.abspath(os.path.dirname(__file__))
        ws_root = current_dir
        while current_dir != "/":
            potential_utils = os.path.join(current_dir, "utils")
            if os.path.exists(os.path.join(potential_utils, "check_status.py")):
                if potential_utils not in sys.path:
                    sys.path.append(potential_utils)
                ws_root = current_dir
                break
            current_dir = os.path.dirname(current_dir)

        try:
            from check_status import connect_and_check

            self.drone = connect_and_check()
        except ImportError as e:
            self.get_logger().error(
                f"Could not import check_status from utils: {e}")
            self.drone = Tello()
            self.drone.connect()

        # Initialize velocities
        self.InercialVel = np.zeros(3)
        self.angularVel = 0
        self.BodyVelocity = np.zeros(3)

        # Load gains from pi.conf
        import configparser

        self.config = configparser.ConfigParser()
        pi_conf_path = os.path.join(ws_root, "config", "pi.conf")

        if os.path.exists(pi_conf_path):
            self.config.read(pi_conf_path)
            try:
                # We strictly require these keys without defaults. If missing, it raises KeyError.
                self.Kp_x = float(self.config["GAINS"]["Kp_x"])
                self.Kp_y = float(self.config["GAINS"]["Kp_y"])
                self.Kp_z = float(self.config["GAINS"]["Kp_z"])
                
                self.Ki_x = float(self.config["GAINS"]["Ki_x"])
                self.Ki_y = float(self.config["GAINS"]["Ki_y"])
                self.Ki_z = float(self.config["GAINS"]["Ki_z"])
                self.max_integral = float(self.config["GAINS"]["max_integral"])
                
                self.Kp_yaw = float(self.config["GAINS"]["Kp_yaw"])
                self.Ki_yaw = float(self.config["GAINS"]["Ki_yaw"])
                
                self.get_logger().info(f"Loaded PI gains -> Kp:({self.Kp_x}, {self.Kp_y}, {self.Kp_z}, yaw:{self.Kp_yaw}) | Ki:({self.Ki_x}, {self.Ki_y}, {self.Ki_z}, yaw:{self.Ki_yaw})")
            except KeyError as e:
                self.get_logger().error(f"CRITICAL: Missing gain {e} in pi.conf. Aborting flight.")
                sys.exit(1)
        else:
            self.get_logger().error(f"CRITICAL: pi.conf not found at {pi_conf_path}. Refusing to fly without gains. Aborting.")
            sys.exit(1)
            
        # Controller state variables
        self.error_sum = np.zeros(3)
        self.error_sum_yaw = 0.0
        self.last_time = None
        self.get_logger().info("WAITING FOR OPTITRACK DATA... (Check Motive connection)")

    def signal_handler(self, sig, frame):
        # Safe shutdown on SIGINT
        print("\n[!] Ctrl+C detected. Stopping and landing...", flush=True)
        try:
            if self.has_taken_off:
                self.drone.send_rc_control(0, 0, 0, 0)
                self.drone.land()
            self.drone.end()
        except Exception as e:
            print(f"Error during shutdown: {e}", flush=True)
        finally:
            rclpy.shutdown()
            sys.exit(0)

    def goal_callback(self, msg):
        # Desired position setpoint
        self.Desired_x = msg.pose.position.x
        self.Desired_y = msg.pose.position.y
        self.Desired_z = msg.pose.position.z
        
        # Desired yaw setpoint
        qx = msg.pose.orientation.x
        qy = msg.pose.orientation.y
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w
        
        # If no orientation is sent (e.g., from send_goal.sh or square_routine), 
        # ROS 2 defaults to [0,0,0,0], which crashes scipy's math. We fix it to neutral.
        if qx == 0.0 and qy == 0.0 and qz == 0.0 and qw == 0.0:
            qw = 1.0
            
        from scipy.spatial.transform import Rotation as R
        _, _, self.Desired_yaw = R.from_quat([qx, qy, qz, qw]).as_euler("xyz", degrees=False)
        
        import math
        yaw_deg = self.Desired_yaw * (180.0 / math.pi)
        self.get_logger().info(f"New objective: Pos({self.Desired_x:.2f}, {self.Desired_y:.2f}, {self.Desired_z:.2f}) Yaw({yaw_deg:.1f} deg)")

    def data_callback(self, msg):
        """OptiTrack telemetry callback."""
        if not hasattr(self, "first_data_received"):
            self.first_data_received = True
            self.get_logger().info("OPTITRACK DATA RECEIVED!")
            
        self.Posex = msg.pose.position.x
        self.Posey = msg.pose.position.y
        self.Posez = msg.pose.position.z

        # Orientation quaternion
        self.Qx = msg.pose.orientation.x
        self.Qy = msg.pose.orientation.y
        self.Qz = msg.pose.orientation.z
        self.Qw = msg.pose.orientation.w

        if not hasattr(self, "Desired_x"):
            if not getattr(self, "waiting_for_goal_printed", False):
                self.get_logger().warn("WAITING FOR A GOAL. Use send_goal.sh!")
                self.waiting_for_goal_printed = True
            return

        if not self.has_taken_off:
            # Auto-takeoff on first setpoint
            self.drone.takeoff()
            self.has_taken_off = True
            self.get_logger().info("Takeoff executed")
            import time
            self.last_time = time.time()

        # log the current pose and orientation of the drone for debugging purposes
        # self.get_logger().info(
        #     f"Pose: ({self.Posex:.3f}, {self.Posey:.3f}, {self.Posez:.3f}) "
        #     f"Quat: ({self.Qx:.3f}, {self.Qy:.3f}, {self.Qz:.3f}, {self.Qw:.3f})"
        # )

        """Principal control loop"""
        self.control_variables()
        self.calculateVelocities()
        self.sendDataToTello()
        self.CheckIfReached()

    def control_variables(self):
        # Extract yaw angle from quaternion
        self.roll, self.pitch, self.yaw = R.from_quat(
            [self.Qx, self.Qy, self.Qz, self.Qw]
        ).as_euler("xyz", degrees=False)
        # convert to matrix form for easier calculations
        self.P = np.array([self.Posex, self.Posey, self.Posez])

        # Target position vector
        self.desired_P = np.array(
            [self.Desired_x, self.Desired_y, self.Desired_z])

        # Inverse rotation matrix (inertial to body frame)
        self.Re_inv = np.array(
            [
                [np.cos(self.yaw), np.sin(self.yaw), 0],
                [-np.sin(self.yaw), np.cos(self.yaw), 0],
                [0, 0, 1],
            ]
        )

        # Position error
        self.Pe = self.P - self.desired_P
        
        # Yaw error (normalized to -pi to pi)
        # Note: Error = Current - Target (same as position logic)
        yaw_diff = self.yaw - self.Desired_yaw
        self.Pe_yaw = np.arctan2(np.sin(yaw_diff), np.cos(yaw_diff))

    def calculateVelocities(self):
        import time
        current_time = time.time()
        
        # Calculate Delta time (dt)
        if self.last_time is None:
            dt = 0.0
        else:
            dt = current_time - self.last_time
        self.last_time = current_time

        # Update integral errors
        self.error_sum += self.Pe * dt
        self.error_sum_yaw += self.Pe_yaw * dt
        
        # Anti-windup clamping
        self.error_sum = np.clip(self.error_sum, -self.max_integral, self.max_integral)
        self.error_sum_yaw = np.clip(self.error_sum_yaw, -self.max_integral, self.max_integral)
        
        # the inercial velocity is calculated in the inertial frame and then transformed to the body frame using the inverse rotation matrix
        # the body velocity is the one that is sent to the drone, so we need to transform it to the body frame
        
        # PI Control Law: V = - (Kp * Error + Ki * Integral)
        self.InercialVel = np.array([
            - (self.Kp_x * self.Pe[0] + self.Ki_x * self.error_sum[0]),
            - (self.Kp_y * self.Pe[1] + self.Ki_y * self.error_sum[1]),
            - (self.Kp_z * self.Pe[2] + self.Ki_z * self.error_sum[2])
        ])

        self.BodyVelocity = self.Re_inv @ self.InercialVel
        
        # Angular Velocity (Yaw)
        # Tello send_rc_control expects positive for Clockwise rotation.
        # Our math output matches CCW/CW perfectly with the Tello SDK when passed raw.
        self.angularVel = int(np.clip(self.Kp_yaw * self.Pe_yaw + self.Ki_yaw * self.error_sum_yaw, -100, 100))

    """send data to tello overwriting the rc"""

    def sendDataToTello(self):
        # self.get_logger().info(
        # f"Pe: {self.Pe}, InercialVel: {self.InercialVel}, BodyVel: {self.BodyVelocity}"
        # )

        # We clip the values to ensure they are within the acceptable range (-100 to 100).
        # In our ROS FLU frame:
        # BodyVelocity[0] is X (Forward)
        # BodyVelocity[1] is Y (Left)
        # BodyVelocity[2] is Z (Up)

        # djitellopy send_rc_control expects: (Left/Right, Forward/Backward, Up/Down, Yaw)
        # It expects positive for Right, so we invert Y (Left).
        lr_command = int(np.clip(-self.BodyVelocity[1], -100, 100))
        fb_command = int(np.clip(self.BodyVelocity[0], -100, 100))
        ud_command = int(np.clip(self.BodyVelocity[2], -100, 100))

        self.drone.send_rc_control(
            lr_command, fb_command, ud_command, self.angularVel)

    def CheckIfReached(self):
        # Check setpoint convergence (15 cm threshold for pos, ~11.5 deg for yaw)
        distance_threshold = 0.15
        yaw_threshold = 0.2

        x_ok = abs(self.P[0] - self.Desired_x) <= distance_threshold
        y_ok = abs(self.P[1] - self.Desired_y) <= distance_threshold
        z_ok = abs(self.P[2] - self.Desired_z) <= distance_threshold
        yaw_ok = abs(self.Pe_yaw) <= yaw_threshold

        # clean print to terminal without ROS logger spam 
        import time

        if not hasattr(self, "last_print_time"):
            self.last_print_time = 0

        if time.time() - self.last_print_time > 0.5:
            import math
            yaw_err_deg = abs(self.Pe_yaw) * (180.0 / math.pi)
            print(f"Error to target -> x: {abs(self.Pe[0]):.3f}m | y: {abs(self.Pe[1]):.3f}m | z: {abs(self.Pe[2]):.3f}m | yaw: {yaw_err_deg:.1f}deg", flush=True)
            self.last_print_time = time.time()

        if x_ok and y_ok and z_ok and yaw_ok:
            self.get_logger().warn("Target reached.")
            msg = Bool()
            msg.data = True
            self.reached_pub.publish(msg)


def main(args=None):
    """main function to run the node"""
    rclpy.init(args=args)
    node = TelloController()

    # Handle Ctrl+C signal to land the drone safely and shutdown ROS2
    signal.signal(signal.SIGINT, lambda sig,
                  frame: node.signal_handler(sig, frame))
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

