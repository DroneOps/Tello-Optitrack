import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt # Library for plotting the data, we will use it to plot the trajectory of the drone and the desired position in 3D space. We will also use it to plot the x, y, z and yaw angles of the drone over time.
from mpl_toolkits.mplot3d import Axes3D # Library for 3D plotting, we will use it to plot the trajectory of the drone in 3D space.
import time

'''This script is used to plot the trajectory of the drone and the desired position in 3D space, as well as the x, y, z and yaw angles over time.
The data is received from the /drone/pose topic, which is published by the natnet_ros2 package that receives the data from the OptiTrack motion capture system. 
The desired position is received from the /goal topic, which is published by the user input node.
The script uses the matplotlib library to plot the data, and it is designed to be run in a Jupyter notebook for better visualization. 
The script also handles the Ctrl+C signal to plot the data'''
class PosePlotter(Node):
    def __init__(self):
        super().__init__('pose_plotter')
        
        # Declare parameter for rigid body name (defaults to 'drone')
        self.declare_parameter('rigid_body_name', 'drone')
        rigid_body_name = self.get_parameter('rigid_body_name').get_parameter_value().string_value
        optitrack_topic = f'/{rigid_body_name}/pose'

        self.subscription = self.create_subscription(PoseStamped, optitrack_topic, self.callback, 10)

         #suscriber to goal topic 
        self.goal_sub = self.create_subscription(
            PoseStamped, "/goal", self.goal_callback, 10
        )

        #initialize lists to store the data
        self.x, self.y, self.z, self.yaw, self.t = [], [], [], [], []
        self.start_time = time.time()
        self.get_logger().info('Ready to Plot.') #send a message to the console to indicate that the node is ready to receive data and plot it.

    def callback(self, msg):
        '''callback function to store the data from the optitrack system, it is called every time a new message is received from the /drone/pose topic.
          It stores the x, y, z and yaw angles of the drone in separate lists, as well as the time elapsed since the start of the program. 
          The data is then used to plot the trajectory of the drone and the desired position in 3D space, as well as the x, y, z and yaw angles over time.'''
        self.x.append(msg.pose.position.x)
        self.y.append(msg.pose.position.y)
        self.z.append(msg.pose.position.z)

        # Cuaternion to Euler angles 
        r = R.from_quat([
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w
        ])
        yaw = r.as_euler('xyz', degrees=False)[2] #we are only interested in the yaw angle
        self.yaw.append(yaw)

        self.t.append(time.time() - self.start_time)
        
    def goal_callback(self, msg):
        #capture the desired position from the goal topic
        self.Desired_x = msg.pose.position.x
        self.Desired_y = msg.pose.position.y
        self.Desired_z = msg.pose.position.z

    def plot(self):
        # ---- Figure 1: x, y, z, yaw vs time ----
        plt.figure(figsize=(10, 8))
        plt.subplot(4, 1, 1); plt.plot(self.t, self.x, label='x'); plt.grid(); plt.legend()
        plt.subplot(4, 1, 2); plt.plot(self.t, self.y, label='y'); plt.grid(); plt.legend()
        plt.subplot(4, 1, 3); plt.plot(self.t, self.z, label='z'); plt.grid(); plt.legend()
        plt.subplot(4, 1, 4); plt.plot(self.t, self.yaw, label='yaw'); plt.grid(); plt.legend()
        plt.xlabel('Time [s]')
        plt.tight_layout()

        # ---- Figure 2: 3D trajectory ----
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.plot(self.x, self.y, self.z, color='blue', label='3D trajectory')
        ax.scatter(self.x[0], self.y[0], self.z[0], color='green', label='Start')
        if hasattr(self, 'Desired_x'):
            ax.scatter(self.Desired_x, self.Desired_y, self.Desired_z, color='black', label='Goal')
        ax.scatter(self.x[-1], self.y[-1], self.z[-1], color='red', label='End')
        ax.set_xlabel('X [m]')
        ax.set_ylabel('Y [m]')
        ax.set_zlabel('Z [m]')
        ax.legend()
        ax.set_title('3D trajectory of the drone')
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
