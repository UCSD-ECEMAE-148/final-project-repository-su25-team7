import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import numpy as np
from sklearn.cluster import DBSCAN

class LidarObjectDetector(Node):
    def __init__(self):
        super().__init__('lidar_object_detector')

        # Subscribe to LaserScan topic
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',  # change if your topic name is different
            self.lidar_callback,
            10)

        self.publisher_ = self.create_publisher(String, 'lidar_alert', 10)
        self.get_logger().info("Lidar Object Detector Node Started")

        self.lane = True  # if True -> RIGHT lane, if false -> LEFT lane. Default = RIGHT lane

        # Define a state machine for the robot's behavior
        # States: 'SEARCHING', 'STOPPING', 'MANEUVERING'
        self.state = 'SEARCHING'

    def lidar_callback(self, msg: LaserScan):
        
        # Define angular ranges for lane filtering
        # Assuming these are defined relative to the sensor's physical setup
        LEFT_ANGLE_MIN_FILTER = (260.0 / 180.0) * np.pi  # 260 degrees
        LEFT_ANGLE_MAX_FILTER = (295.0 / 180.0) * np.pi  # 295 degrees
        RIGHT_ANGLE_MIN_FILTER = (245.0 / 180.0) * np.pi  # 245 degrees
        RIGHT_ANGLE_MAX_FILTER = (280.0 / 180.0) * np.pi  # 280 degrees
        
        # The new forward angle is 270 degrees, which is -pi/2 in standard radians
        FORWARD_RAD = (270.0 / 180.0) * np.pi
        
        # Define an angular tolerance for objects considered "straight ahead"
        FORWARD_TOLERANCE = (5.0 / 180.0) * np.pi  # 5 degrees tolerance
        
        # State 1: SEARCHING for an object
        if self.state == 'SEARCHING':
            # Check for objects in the current lane
            if self.lane:  # Currently in the right lane
                for i, r in enumerate(msg.ranges):
                    if np.isfinite(r):
                        angle = msg.angle_min + i * msg.angle_increment
                        # Filter by angular window for the right lane
                        if RIGHT_ANGLE_MIN_FILTER <= angle <= RIGHT_ANGLE_MAX_FILTER:
                            if 2.0 <= r <= 2.2:
                                self.state = 'STOPPING'
                                self.get_logger().info("Obstacle detected, STOPPING robot.")
                                # Publish a message to stop the robot
                                self.publisher_.publish(String(data="STOP"))
                                return
            else:  # Currently in the left lane
                for i, r in enumerate(msg.ranges):
                    if np.isfinite(r):
                        angle = msg.angle_min + i * msg.angle_increment
                        # Filter by angular window for the left lane
                        if LEFT_ANGLE_MIN_FILTER <= angle <= LEFT_ANGLE_MAX_FILTER:
                            if 2.0 <= r <= 2.2:
                                self.state = 'STOPPING'
                                self.get_logger().info("Obstacle detected, STOPPING robot.")
                                # Publish a message to stop the robot
                                self.publisher_.publish(String(data="STOP"))
                                return
        
        # State 2: STOPPING to evaluate the situation
        elif self.state == 'STOPPING':
            # The robot is stopped. Now, perform the object detection and decision logic once.
            
            points = []
            if self.lane: #if currently in right lane
                angle_min = RIGHT_ANGLE_MIN_FILTER
                angle_max = RIGHT_ANGLE_MAX_FILTER
            else: #if currently in left lane
                angle_min = LEFT_ANGLE_MIN_FILTER
                angle_max = LEFT_ANGLE_MAX_FILTER

            for i, r in enumerate(msg.ranges):
                if np.isfinite(r):
                    angle = msg.angle_min + i * msg.angle_increment
                    if angle_min <= angle <= angle_max and r <= 2.2:
                        x = r * np.cos(angle)
                        y = r * np.sin(angle)
                        points.append([x, y])

            if not points:
                # If no points are found, something is wrong. Return to SEARCHING.
                self.get_logger().warn("No points found during evaluation. Returning to SEARCHING state.")
                self.state = 'SEARCHING'
                return

            points = np.array(points)
            clustering = DBSCAN(eps=0.035, min_samples=10).fit(points)
            labels = clustering.labels_

            objects = []
            for cluster_id in set(labels):
                if cluster_id == -1:  # skip noise
                    continue
                cluster_points = points[labels == cluster_id]
                centroid = np.mean(cluster_points, axis=0)
                distance = np.linalg.norm(centroid)
                angle = np.arctan2(centroid[1], centroid[0])
                
                # Convert the angle to be in the [0, 2*pi] range for easier comparison
                if angle < 0:
                    angle += 2 * np.pi
                
                objects.append((cluster_id, distance, angle))

            if not objects:
                # No objects confirmed, resume search.
                self.get_logger().info("No objects confirmed. Resuming search.")
                self.state = 'SEARCHING'
                return

            # Decision logic based on object count and location
            if len(objects) == 1:
                # Single object detected
                obj = objects[0]
                self.get_logger().info(f"One object detected at {np.degrees(obj[2]):.1f}°")
                
                # Check if the object is within the forward tolerance range
                is_forward = abs(obj[2] - FORWARD_RAD) < FORWARD_TOLERANCE

                if self.lane:  # Currently in the right lane
                    if is_forward:
                        # Object is directly in front, must change lanes
                        self.get_logger().info("Object directly in front, moving to LEFT LANE.")
                        self.publisher_.publish(String(data="CHANGE_LANE_LEFT"))
                        self.lane = False
                        # Transition to MANEUVERING state to prevent re-execution during movement
                        self.state = 'MANEUVERING'
                        
                    elif obj[2] < FORWARD_RAD:  # Object is to the right of forward
                        # Object is in our lane, must change lanes
                        self.get_logger().info("Object in front on the right, moving to LEFT LANE.")
                        self.publisher_.publish(String(data="CHANGE_LANE_LEFT"))
                        self.lane = False
                        # Transition to MANEUVERING state to prevent re-execution during movement
                        self.state = 'MANEUVERING'
                        
                        #set timer for node to pause taking new data for 5 seconds
                        self.timer = self.create_timer(5.0, self.resume_searching_callback)
                        
                    else:  # Object is to the left of forward (in the left lane)
                        # Our path is clear, continue forward
                        self.get_logger().info("Object is on the other side, continuing FORWARD.")
                        self.publisher_.publish(String(data="GO_STRAIGHT"))
                else:  # Currently in the left lane
                    if is_forward:
                        # Object is directly in front, must change lanes
                        self.get_logger().info("Object directly in front, moving to RIGHT LANE.")
                        self.publisher_.publish(String(data="CHANGE_LANE_RIGHT"))
                        self.lane = True
                        # Transition to MANEUVERING state to prevent re-execution during movement
                        self.state = 'MANEUVERING'
                        
                        #set timer for node to pause taking new data for 5 seconds
                        self.timer = self.create_timer(5.0, self.resume_searching_callback)
                        
                    elif obj[2] > FORWARD_RAD:  # Object is to the left of forward
                        # Object is in our lane, must change lanes
                        self.get_logger().info("Object in front on the left, moving to RIGHT LANE.")
                        self.publisher_.publish(String(data="CHANGE_LANE_RIGHT"))
                        self.lane = True
                        # Transition to MANEUVERING state to prevent re-execution during movement
                        self.state = 'MANEUVERING'
                        
                        #set timer for node to pause taking new data for 5 seconds
                        self.timer = self.create_timer(5.0, self.resume_searching_callback)
                        
                    
                    else:  # Object is to the right of forward (in the right lane)
                        # Our path is clear, continue forward
                        self.get_logger().info("Object is on the other side, continuing FORWARD.")
                        self.publisher_.publish(String(data="GO_STRAIGHT"))
            
            else: # Multiple objects detected: dead end
                self.get_logger().info("DEAD END detected, initiating U-turn.")
                if self.lane:
                    self.publisher_.publish(String(data="U_TURN_LEFT"))
                    # Transition to MANEUVERING state to prevent re-execution during movement
                    self.state = 'MANEUVERING'
                    
                    #set timer for node to pause taking new data for 5 seconds
                    self.timer = self.create_timer(5.0, self.resume_searching_callback)
                    
                else:
                    self.publisher_.publish(String(data="U_TURN_RIGHT"))
                    # Transition to MANEUVERING state to prevent re-execution during movement
                    self.state = 'MANEUVERING'
                    
                    #set timer for node to pause taking new data for 5 seconds
                    self.timer = self.create_timer(5.0, self.resume_searching_callback)

        # State 3: MANEUVERING, ignore new LiDAR data until the maneuver is complete
        elif self.state == 'MANEUVERING':
            pass


def main():
    rclpy.init()
    node = LidarObjectDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
