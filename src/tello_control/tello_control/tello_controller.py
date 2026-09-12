import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from djitellopy import Tello # Library to control the Tello drone, it is a wrapper around the official SDK of the drone that allows us to send commands to the drone and receive data from it.
from scipy.spatial.transform import Rotation as R
import numpy as np
import signal
import sys

'''This script is used to control the Tello drone in a cinematic way, it receives the current position and orientation of the drone from the OptiTrack motion capture system
 through the /drone/pose topic, and it receives the desired position from the user input through the /goal topic. 
 The script uses a simple proportional controller to calculate the velocity that the drone should have in order to reach the desired position, 
 and it sends this velocity to the drone using the send_rc_control method of the djitellopy library. 
 Rotation Matrix is used to transform the velocity from the inertial frame to the body frame of the drone, which is the frame that the drone uses to control its movement.'''

class TelloController(Node):
    def __init__(self):
        super().__init__('tello_controller')
        
        # Declare parameter for rigid body name (defaults to 'drone')
        self.declare_parameter('rigid_body_name', 'drone')
        self.rigid_body_name = self.get_parameter('rigid_body_name').get_parameter_value().string_value

        #flag
        self.has_taken_off = False

        # ros2 suscriber to optitrack data
        optitrack_topic = f'/{self.rigid_body_name}/pose'
        self.subscription = self.create_subscription(
            PoseStamped, optitrack_topic, #topic name from natnet_ros2 package
            self.data_callback,
            10
        )
        #suscriber to goal topic 
        self.goal_sub = self.create_subscription(
            PoseStamped, "/goal", self.goal_callback, 10
        )

        # publisher for goal reached
        self.reached_pub = self.create_publisher(Bool, '/goal_reached', 10)

        # Connect to Tello using the utils check_status script
        import sys
        import os
        # Add the utils folder to sys.path to import check_status
        # The node is in src/tello_control/tello_control/
        # so utils is 4 levels up from this file's directory
        utils_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../../utils"))
        if os.path.exists(utils_path) and utils_path not in sys.path:
            sys.path.append(utils_path)
            
        try:
            from check_status import connect_and_check
            self.drone = connect_and_check()
        except ImportError:
            self.get_logger().error("Could not import check_status from utils. Check paths.")
            self.drone = Tello()
            self.drone.connect()
            
        # Initialize velocities
        self.InercialVel = np.zeros(3)
        self.angularVel = 0
        self.BodyVelocity = np.zeros(3)

        '''desired values and constant'''
        self.Kp = 0.5
        self.kp_z = 1
    
    def signal_handler(self, sig, frame):
        # Handle Ctrl+C signal to land the drone safely and shutdown ROS2
        print("\n[!] Ctrl+C detected. Stopping and landing...")
        try:
            if self.has_taken_off:
                self.drone.send_rc_control(0, 0, 0, 0)
                self.drone.land()
            self.drone.end()
        except Exception as e:
            print(f"Error during shutdown: {e}")
        finally:
            rclpy.shutdown()
            sys.exit(0)

    def goal_callback(self, msg):
        #capture the desired position from the goal topic from the user input
        self.Desired_x = msg.pose.position.x
        self.Desired_y = msg.pose.position.y
        self.Desired_z = msg.pose.position.z
        self.get_logger().info(f"New objective: ({self.Desired_x}, {self.Desired_y}, {self.Desired_z})") #log the new goal for debugging purposes

   
    def data_callback(self, msg):
        '''callback function to capture the current position and orientation of the drone from optitrack data'''
        self.Posex = msg.pose.position.x
        self.Posey = msg.pose.position.y
        self.Posez = msg.pose.position.z

        #get orientation in quaternion format
        #this is the format that optitrack gives us, but we will convert it to euler angles later for the control loop
        self.Qx = msg.pose.orientation.x
        self.Qy = msg.pose.orientation.y
        self.Qz = msg.pose.orientation.z
        self.Qw = msg.pose.orientation.w

        if not hasattr(self, 'Desired_x'):
            return 

        if not self.has_taken_off:
            #takeoff the drone if it has not taken this is the first time we receive data and we have a goal
            #this is done to avoid taking off before we have a goal and to ensure that we have the initial position of the drone before taking off
            self.drone.takeoff()
            self.has_taken_off = True
            self.get_logger().info("Takeoff executed")
        
        
        #log the current pose and orientation of the drone for debugging purposes
        self.get_logger().info(
            f"Pose: ({self.Posex:.3f}, {self.Posey:.3f}, {self.Posez:.3f}) "
            f"Quat: ({self.Qx:.3f}, {self.Qy:.3f}, {self.Qz:.3f}, {self.Qw:.3f})"
        )


        '''Principal control loop'''
        #calculate variables
        self.control_variables()
        #calculate velocity
        self.calculateVelocities()
        #send data after processing
        self.sendDataToTello()
        #check if it is close enough to the target
        self.CheckIfReached()

    '''control variables'''
    def control_variables(self):
        # Convert quaternion to Euler angles to get the yaw angle of the drone, which is needed for the control loop. 
        self.roll, self.pitch, self.yaw = R.from_quat([self.Qx, self.Qy, self.Qz, self.Qw]).as_euler('xyz', degrees=False)
        #convert to matrix form for easier calculations
        self.P = np.array([
            self.Posex,
            self.Posey,
            self.Posez
        ])
        
        #desired Matrix for the control loop
        self.desired_P = np.array([
            self.Desired_x,
            self.Desired_y,
            self.Desired_z
        ])

         #inverse rotation matrix to transform from inertial frame to body frame based on the current yaw angle of the drone
         #  we only consider the yaw angle for the rotation since the drone is assumed to be always parallel to the ground (no roll and pitch)
         #  and we want to control the velocity in the horizontal plane (x and y) and the vertical velocity (z) independently.
        self.Re_inv = np.array([
            [np.cos(self.yaw), np.sin(self.yaw), 0],
            [-np.sin(self.yaw),  np.cos(self.yaw), 0],
            [0,                0,               1]
        ])

        #calculate error
        self.Pe = self.P - self.desired_P 
       

    '''calculate inercial velocity'''
    def calculateVelocities(self):
        #the inercial velocity is calculated in the inertial frame and then transformed to the body frame using the inverse rotation matrix
        #the body velocity is the one that is sent to the drone, so we need to transform it to the body frame
        self.InercialVel = np.array([
        -self.Kp * self.Pe[0],
        -self.Kp * self.Pe[1],
        -self.kp_z * self.Pe[2]
        ])

        self.BodyVelocity = self.Re_inv @ self.InercialVel
    
    '''send data to tello overwriting the rc'''
    def sendDataToTello(self):
        self.get_logger().info(
        f"Pe: {self.Pe}, InercialVel: {self.InercialVel}, BodyVel: {self.BodyVelocity}"
        )
        #we use a scale factor to convert the velocity from m/s to the range of -100 to 100 that the tello accepts for rc control. We also clip the values to ensure they are within the acceptable range.

        scale = 20
        vx = int(np.clip(self.BodyVelocity[0]*scale, -100, 100))
        vy = int(np.clip(self.BodyVelocity[1]*scale, -100, 100))
        vz = int(np.clip(self.BodyVelocity[2]*scale, -100, 100))
        self.drone.send_rc_control(vx, vy, vz, self.angularVel) 


    def CheckIfReached(self):
        # Check if the drone is close enough to the target position
        # We use a 15 cm threshold 
        distance_threshold = 0.15

        x_ok = abs(self.P[0] - self.Desired_x) <= distance_threshold
        y_ok = abs(self.P[1] - self.Desired_y) <= distance_threshold
        z_ok = abs(self.P[2] - self.Desired_z) <= distance_threshold

        # log the distance to the target position for debugging purposes
        self.get_logger().info(
            f"Distance to target: x: {abs(self.P[0] - self.Desired_x):.3f}, y: {abs(self.P[1] - self.Desired_y):.3f}, z: {abs(self.P[2] - self.Desired_z):.3f}"
        )
        
        if x_ok and y_ok and z_ok:
            self.get_logger().warn("Target reached. Publishing flag...")
            msg = Bool()
            msg.data = True
            self.reached_pub.publish(msg)

def main(args=None):
    '''main function to run the node'''
    rclpy.init(args=args)
    node = TelloController()

    # Handle Ctrl+C signal to land the drone safely and shutdown ROS2
    signal.signal(signal.SIGINT, lambda sig, frame: node.signal_handler(sig, frame)) 
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__== '__main__':
    main()