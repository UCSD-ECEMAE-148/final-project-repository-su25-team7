import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import Twist

class PriorityTwistNode(Node):
    def __init__(self):
        super().__init__('vesc_priority_node')

        # Subscribe to Lane Detection and LiDAR topics
        self.lane_detect_subscription = self.create_subscription(
            Twist,
            '/cmd_vel/team0',
            self.lane_detect_twist_callback,
            10)
        self.lidar_subscription = self.create_subscription(
            Twist,
            '/cmd_vel/team1',
            self.lidar_twist_callback,
            10)

        # publish to the VESC directly
        self.twist_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Stored messages and timestamps
        self.lidar_twist_msg = Twist()
        self.lane_twist_msg = Twist()
        self.last_msg_time_lidar = self.get_clock().now()
        self.last_msg_time_lane = self.get_clock().now()
        self.active_controller = 'none' # State variable for logging

        # Timeout threshold for the LiDAR topic
        self.timeout_threshold = Duration(seconds=1)

        # Main arbitration timer
        self.timer = self.create_timer(0.1, self.arbitration_callback)

        self.get_logger().info("VESC Priority Node Started.")

    def lidar_twist_callback(self, msg):
        self.lidar_twist_msg = msg
        self.last_msg_time_lidar = self.get_clock().now()
        self.get_logger().info('Received message from TEAM7_LIDAR, updating timestamp.')

    def lane_detect_twist_callback(self, msg):
        self.lane_twist_msg = msg
        self.last_msg_time_lane = self.get_clock().now()
        self.get_logger().info('Received message from LANE_DETECTION, updating timestamp.')

    def arbitration_callback(self):
        current_time = self.get_clock().now()

        # Check if LiDAR has timed out
        if (current_time - self.last_msg_time_lidar) < self.timeout_threshold:
            # LiDAR is active, publish its message
            if self.active_controller != 'lidar':
                self.get_logger().info('Switching to LiDAR control.')
                self.active_controller = 'lidar'
            self.twist_publisher.publish(self.lidar_twist_msg)

        # Check if Lane Detection has timed out
        elif (current_time - self.last_msg_time_lane) < self.timeout_threshold:
            # LiDAR timed out, but Lane Detection is active
            if self.active_controller != 'lane_detection':
                self.get_logger().info('LiDAR timed out. Switching to Lane Detection control.')
                self.active_controller = 'lane_detection'
            self.twist_publisher.publish(self.lane_twist_msg)
        else:
            # Both topics have timed out, publish a zero Twist message to stop the robot
            if self.active_controller != 'none':
                self.get_logger().warn('Both control topics timed out. Stopping the robot.')
                self.active_controller = 'none'
            stop_twist = Twist()
            self.twist_publisher.publish(stop_twist)

def main(args=None):
    rclpy.init(args=args)
    node = PriorityTwistNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()