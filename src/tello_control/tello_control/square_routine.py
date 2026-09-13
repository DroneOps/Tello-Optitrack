"""
Autonomous flight routine node.
Sequentially publishes square waypoints to /goal and waits for /goal_reached confirmation.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool

class SquareRoutine(Node):
    def __init__(self):
        super().__init__('square_routine')
        
        self.goal_pub = self.create_publisher(PoseStamped, '/goal', 10)
        self.reached_sub = self.create_subscription(Bool, '/goal_reached', self.reached_callback, 10)
        
        # Waypoint wait time parameter
        self.declare_parameter('wait_time', 2.0)
        self.wait_time = self.get_parameter('wait_time').value
        
        # Square trajectory waypoints
        self.waypoints = [
            (1.0, 1.0, 1.0),
            (1.0, -1.0, 1.0),
            (-1.0, -1.0, 1.0),
            (-1.0, 1.0, 1.0),
            (0.0, 0.0, 1.0)
        ]
        self.current_idx = 0
        self.waiting_for_reach = False
        
        # Initialization delay timer
        self.timer = self.create_timer(2.0, self.start_routine)
        
    def start_routine(self):
        self.timer.cancel()
        self.get_logger().info("Starting square routine...")
        self.send_waypoint()

    def send_waypoint(self):
        if self.current_idx < len(self.waypoints):
            wp = self.waypoints[self.current_idx]
            msg = PoseStamped()
            msg.pose.position.x = wp[0]
            msg.pose.position.y = wp[1]
            msg.pose.position.z = wp[2]
            
            self.goal_pub.publish(msg)
            self.get_logger().info(f"Sent Waypoint {self.current_idx+1}: {wp}")
            self.waiting_for_reach = True
        else:
            self.get_logger().info("Square routine completed! Hovering at center.")

    def reached_callback(self, msg):
        # Process waypoint reached signal
        if msg.data and self.waiting_for_reach:
            self.get_logger().info(f"Waypoint reached! Stabilizing for {self.wait_time} seconds...")
            self.waiting_for_reach = False
            self.current_idx += 1
            
            # Waypoint transition timer
            self.wait_timer = self.create_timer(self.wait_time, self.next_wp_callback)

    def next_wp_callback(self):
        self.wait_timer.cancel()
        self.send_waypoint()

def main(args=None):
    rclpy.init(args=args)
    node = SquareRoutine()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
