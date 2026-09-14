"""
Real-time pose plotting and telemetry logger.
Subscribes to OptiTrack feedback and generates trajectory plots on shutdown.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt
import time
import os
from datetime import datetime

class PosePlotter(Node):
    def __init__(self):
        super().__init__('pose_plotter')
        
        # Rigid body parameter
        self.declare_parameter('rigid_body_name', 'drone')
        rigid_body_name = self.get_parameter('rigid_body_name').get_parameter_value().string_value
        optitrack_topic = f'/{rigid_body_name}/pose'

        self.subscription = self.create_subscription(PoseStamped, optitrack_topic, self.callback, 10)

        # Goal subscriber
        self.goal_sub = self.create_subscription(
            PoseStamped, "/goal", self.goal_callback, 10
        )

        # Telemetry data buffers
        self.x, self.y, self.z, self.yaw, self.t = [], [], [], [], []
        self.start_time = time.time()
        self.get_logger().info('Ready to Plot.')

    def callback(self, msg):
        '''callback function to store the data from the optitrack system, it is called every time a new message is received from the /drone/pose topic.
          It stores the x, y, z and yaw angles of the drone in separate lists, as well as the time elapsed since the start of the program. 
          The data is then used to plot the trajectory of the drone and the desired position in 3D space, as well as the x, y, z and yaw angles over time.'''
        self.x.append(msg.pose.position.x)
        self.y.append(msg.pose.position.y)
        self.z.append(msg.pose.position.z)

        # Quaternion to Euler angles
        r = R.from_quat([
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w
        ])
        yaw = r.as_euler('xyz', degrees=False)[2]
        self.yaw.append(yaw)

        self.t.append(time.time() - self.start_time)
        
    def goal_callback(self, msg):
        # Desired position setpoint
        self.Desired_x = msg.pose.position.x
        self.Desired_y = msg.pose.position.y
        self.Desired_z = msg.pose.position.z

    def plot(self):
        import numpy as np
        plt.style.use('ggplot')
        
        # ---- Figure 1: x, y, z, yaw vs time ----
        fig1, axs = plt.subplots(4, 1, figsize=(10, 10), sharex=True)
        fig1.suptitle('Drone Telemetry vs Time', fontsize=16)
        
        axs[0].plot(self.t, self.x, color='r', linewidth=2, label='Current X')
        if hasattr(self, 'Desired_x'):
            axs[0].axhline(y=self.Desired_x, color='k', linestyle='--', label='Goal X')
        axs[0].set_ylabel('X [m]')
        axs[0].legend(loc='upper right')
        
        axs[1].plot(self.t, self.y, color='g', linewidth=2, label='Current Y')
        if hasattr(self, 'Desired_y'):
            axs[1].axhline(y=self.Desired_y, color='k', linestyle='--', label='Goal Y')
        axs[1].set_ylabel('Y [m]')
        axs[1].legend(loc='upper right')

        axs[2].plot(self.t, self.z, color='b', linewidth=2, label='Current Z')
        if hasattr(self, 'Desired_z'):
            axs[2].axhline(y=self.Desired_z, color='k', linestyle='--', label='Goal Z')
        axs[2].set_ylabel('Z [m]')
        axs[2].legend(loc='upper right')

        axs[3].plot(self.t, self.yaw, color='m', linewidth=2, label='Yaw')
        axs[3].set_ylabel('Yaw [rad]')
        axs[3].set_xlabel('Time [s]')
        axs[3].legend(loc='upper right')
        
        plt.tight_layout()

        # ---- Figure 2: 3D trajectory ----
        fig2 = plt.figure(figsize=(8, 8))
        ax = fig2.add_subplot(111, projection='3d')
        ax.plot(self.x, self.y, self.z, color='blue', linewidth=2, label='3D trajectory')
        ax.scatter(self.x[0], self.y[0], self.z[0], color='green', s=100, label='Start')
        if hasattr(self, 'Desired_x'):
            ax.scatter(self.Desired_x, self.Desired_y, self.Desired_z, color='black', marker='X', s=150, label='Goal')
        ax.scatter(self.x[-1], self.y[-1], self.z[-1], color='red', s=100, label='End')
        
        ax.set_xlabel('X [m]')
        ax.set_ylabel('Y [m]')
        ax.set_zlabel('Z [m]')
        ax.set_title('3D Trajectory', fontsize=16)
        
        # Force Equal Aspect Ratio for 3D plot to prevent bad plots
        try:
            max_range = np.array([max(self.x)-min(self.x), max(self.y)-min(self.y), max(self.z)-min(self.z)]).max() / 2.0
            mid_x = (max(self.x)+min(self.x)) * 0.5
            mid_y = (max(self.y)+min(self.y)) * 0.5
            mid_z = (max(self.z)+min(self.z)) * 0.5
            ax.set_xlim(mid_x - max_range, mid_x + max_range)
            ax.set_ylim(mid_y - max_range, mid_y + max_range)
            ax.set_zlim(mid_z - max_range, mid_z + max_range)
            ax.set_box_aspect([1,1,1])
        except Exception:
            pass
            
        ax.legend()
        

        
        # Find the root of the workspace to locate the docs folder
        current_dir = os.path.abspath(os.path.dirname(__file__))
        ws_root = current_dir
        while ws_root != "/":
            if os.path.exists(os.path.join(ws_root, "docs")):
                break
            ws_root = os.path.dirname(ws_root)
            
        save_dir = os.path.join(ws_root, 'docs', 'controller_plots')
        os.makedirs(save_dir, exist_ok=True)
            
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        file1 = os.path.join(save_dir, f"telemetry_{timestamp}.png")
        file2 = os.path.join(save_dir, f"trajectory3d_{timestamp}.png")
        
        fig1.savefig(file1, bbox_inches='tight')
        fig2.savefig(file2, bbox_inches='tight')
        
        print("\n" + "="*55)
        print("[+] PLOTS SAVED SUCCESSFULLY!")
        print(f"    -> {file1}")
        print(f"    -> {file2}")
        print("="*55 + "\n")
        
        plt.show()


def main(args=None):
    rclpy.init(args=args)
    node = PosePlotter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.plot()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
