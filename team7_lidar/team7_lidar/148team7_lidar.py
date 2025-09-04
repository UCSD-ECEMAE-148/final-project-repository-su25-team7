import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
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

        self.twist_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.get_logger().info("Lidar Object Detector Node Started")

        self.lane = True  # if True -> RIGHT lane, if false -> LEFT lane. Default = RIGHT lane
        self.twist_cmd = Twist()

        # Define a state machine for the robot's behavior
        # States: 
        # 'SEARCHING', -> search for obstacle in 2m distance
        # 'STOPPING', -> after finding obstacle, determine what action to take
        # 'CHANGE_LANE_LEFT', -> publish to vesc to make robot change to left lane
        # 'CHANGE_LANE_RIGHT', -> publish to vesc to make robot change to right lane
        # 'STOP', -> publish to vesc to make robot stop
        # 'U_TURN_RIGHT', -> publish to vesc to make robot make a U-turn into right lane
        # 'U_TURN_LEFT' -> publish to vesc to make robot make a U-turn into left lane
        self.state = 'SEARCHING'

    def lidar_callback(self, msg: LaserScan):
        # Define angular ranges for lane filtering
        # Assuming these are defined relative to the sensor's physical setup``
        LEFT_ANGLE_MIN_FILTER = (84.0 / 180.0) * np.pi  # 84 degrees
        LEFT_ANGLE_MAX_FILTER = (110.0 / 180.0) * np.pi  # 110 degrees
        RIGHT_ANGLE_MIN_FILTER = (70.0 / 180.0) * np.pi  # 70 degrees
        RIGHT_ANGLE_MAX_FILTER = (96.0 / 180.0) * np.pi  # 96 degrees
        
        # State 1: SEARCHING for an object
        if self.state == 'SEARCHING':
            # Check for objects in the current lane
            if self.lane:  # Currently in the right lane
                for i, r in enumerate(msg.ranges):
                    if np.isfinite(r):
                        angle = msg.angle_min + i * msg.angle_increment
                        # Filter by angular window for the right lane
                        if RIGHT_ANGLE_MIN_FILTER <= angle <= RIGHT_ANGLE_MAX_FILTER:
                            if 2.5 <= r <= 2.7:
                                self.state = 'STOP'
                                self.get_logger().info("Obstacle detected, STOPPING robot.")
                                # Publish a message to stop the robot
                                #self.timer = self.create_timer(5.0, self.resume_searching_callback)
                                
                                return
            else:  # Currently in the left lane
                for i, r in enumerate(msg.ranges):
                    if np.isfinite(r):
                        angle = msg.angle_min + i * msg.angle_increment
                        # Filter by angular window for the left lane
                        if LEFT_ANGLE_MIN_FILTER <= angle <= LEFT_ANGLE_MAX_FILTER:
                            if 2.5 <= r <= 2.7:
                                self.state = 'STOP'
                                self.get_logger().info("Obstacle detected, STOPPING robot.")
                                # Publish a message to stop the robot
                                #self.publisher_.publish(String(data="STOP"))
                                return
        
        # State 2: STOPPING to evaluate the situation
        elif self.state == 'STOPPING':
            # The robot is stopped. Now, perform the object detection and decision logic once.
            
            right_points = []
            left_points = []
            if self.lane: #if currently in right lane
                angle_min = RIGHT_ANGLE_MIN_FILTER
                angle_max = RIGHT_ANGLE_MAX_FILTER
                angle_middle = (RIGHT_ANGLE_MIN_FILTER + RIGHT_ANGLE_MAX_FILTER) / 2
                
                for i, r in enumerate(msg.ranges):
                    if np.isfinite(r):
                        angle = msg.angle_min + i * msg.angle_increment
                        if angle_min <= angle <= angle_middle and 2.5 <= r <= 2.7:
                            x = r * np.cos(angle)
                            y = r * np.sin(angle)
                            left_points.append([x, y])  
                            
                        elif angle_middle <= angle <= angle_max and 2.5 <= r <= 2.7:
                            x = r * np.cos(angle)
                            y = r * np.sin(angle)
                            right_points.append([x, y]) 
                            
                if not left_points:     #if left lane is empty
                    # Object is directly in front, must change lanes
                    self.get_logger().info("Object in front, switching to LEFT LANE.")
                    # self.publisher_.publish(String(data="CHANGE_LANE_LEFT"))
                    self.lane = False
                    # Transition to MANEUVERING state to prevent re-execution during movement
                    self.state = 'CHANGE_LANE_LEFT'    
                    
                elif not right_points:
                    # Our path is clear, continue forward (no VESC publishing)
                    self.get_logger().info("Object is on the other side, continuing FORWARD: right_lane.")
                    self.state = 'MOVE_FORWARD'      
                    
                else: 
                    '''
                    Person Detection Code runs here before U-turn.
                    '''
                    
                    
                    self.get_logger().info("DEAD END detected, initiating U-turn.")
                    self.get_logger().info("Making U-turn left.")
                    # self.publisher_.publish(String(data="U_TURN_LEFT"))
                    # Transition to MANEUVERING state to prevent re-execution during movement
                    self.state = 'U_TURN_LEFT'
                    
                    #set timer for node to pause taking new data for 5 seconds
                    #self.timer = self.create_timer(5.0, self.resume_searching_callback)
                              
                    
            else: #if currently in left lane
                angle_min = LEFT_ANGLE_MIN_FILTER
                angle_max = LEFT_ANGLE_MAX_FILTER
                angle_middle = (LEFT_ANGLE_MIN_FILTER + LEFT_ANGLE_MAX_FILTER) / 2
                
                for i, r in enumerate(msg.ranges):
                    if np.isfinite(r):
                        angle = msg.angle_min + i * msg.angle_increment
                        if angle_min <= angle <= angle_middle and 2.5 <= r <= 2.7:
                            x = r * np.cos(angle)
                            y = r * np.sin(angle)
                            left_points.append([x, y])
                        if angle_middle <= angle <= angle_max and 2.5 <= r <= 2.7:
                            x = r * np.cos(angle)
                            y = r * np.sin(angle)
                            right_points.append([x, y])
                            
                if not left_points:
                    # Our path is clear, continue forward (no VESC publishing)
                    self.get_logger().info("Object is on the other side, continuing FORWARD: left_lane.")
                    self.state = 'MOVE_FORWARD'
                    
                     
                elif not right_points:
                    # Object is directly in front, must change lanes
                    self.get_logger().info("Object in front, switching to RIGHT LANE.")
                    # self.publisher_.publish(String(data="CHANGE_LANE_LEFT"))
                    self.lane = True
                    # Transition to MANEUVERING state to prevent re-execution during movement
                    self.state = 'CHANGE_LANE_RIGHT' 
                else:
                    '''
                    Person Detection Code runs here before U-turn.
                    '''
                    
                    
                    self.get_logger().info("DEAD END detected, initiating U-turn.")
                    self.get_logger().info("Making U-turn Right.")
                    # self.publisher_.publish(String(data="U_TURN_LEFT"))
                    # Transition to MANEUVERING state to prevent re-execution during movement
                    self.state = 'U_TURN_RIGHT'
                    
                    #set timer for node to pause taking new data for 5 seconds
                    #self.timer = self.create_timer(5.0, self.resume_searching_callback)
    
        
        elif self.state == 'CHANGE_LANE_LEFT':
            start_time = self.get_clock().now()
            end_time = self.get_clock().now()
            time_difference = end_time - start_time
            seconds_diff = time_difference.nanoseconds / 1e9
            
            while(seconds_diff < 2):     #activate for 2 seconds 
                # publish -> angular.z 90 degrees right
                self.twist_cmd.linear.x = 0.2
                self.twist_cmd.angular.z = -0.3
                # Publish the message
                self.twist_publisher.publish(self.twist_cmd)
                # Log the published message for verification.
                #self.get_logger().info(f'Publishing: Linear.x="{self.twist_cmd.linear.x:.2f}" Angular.z="{self.twist_cmd.angular.z:.2f}"')
            
                end_time = self.get_clock().now()
                time_difference = end_time - start_time
                seconds_diff = time_difference.nanoseconds / 1e9

            start_time = self.get_clock().now()
            seconds_diff = 0
            while(seconds_diff < 1):     #activate for 2 seconds 
                # publish -> angular.z 90 degrees right
                self.twist_cmd.linear.x = 0.2
                self.twist_cmd.angular.z = 0.4
                # Publish the message.
                self.twist_publisher.publish(self.twist_cmd)
                # Log the published message for verification.
                #self.get_logger().info(f'Publishing: Linear.x="{self.twist_cmd.linear.x:.2f}" Angular.z="{self.twist_cmd.angular.z:.2f}"')
            
                end_time = self.get_clock().now()
                time_difference = end_time - start_time
                seconds_diff = time_difference.nanoseconds / 1e9
              
            self.state = 'SEARCHING'
        
        elif self.state == 'CHANGE_LANE_RIGHT':
            start_time = self.get_clock().now()
            end_time = self.get_clock().now()
            time_difference = end_time - start_time
            seconds_diff = time_difference.nanoseconds / 1e9
            
            while(seconds_diff < 2):     #activate for 2 seconds 
                # publish -> angular.z 90 degrees right
                self.twist_cmd.linear.x = 0.2
                self.twist_cmd.angular.z = 0.6
                # Publish the message.
                self.twist_publisher.publish(self.twist_cmd)
                # Log the published message for verification.
                #self.get_logger().info(f'Publishing: Linear.x="{self.twist_cmd.linear.x:.2f}" Angular.z="{self.twist_cmd.angular.z:.2f}"')
            
                end_time = self.get_clock().now()
                time_difference = end_time - start_time
                seconds_diff = time_difference.nanoseconds / 1e9

            start_time = self.get_clock().now()
            seconds_diff = 0
            while(seconds_diff < 1):     #activate for 2 seconds 
                # publish -> angular.z 90 degrees right
                self.twist_cmd.linear.x = 0.2
                self.twist_cmd.angular.z = -0.2
                # Publish the message.
                self.twist_publisher.publish(self.twist_cmd)
                # Log the published message for verification.
                #self.get_logger().info(f'Publishing: Linear.x="{self.twist_cmd.linear.x:.2f}" Angular.z="{self.twist_cmd.angular.z:.2f}"')
            
                end_time = self.get_clock().now()
                time_difference = end_time - start_time
                seconds_diff = time_difference.nanoseconds / 1e9
                
            self.state = 'SEARCHING'
                
        elif self.state == 'STOP':
            start_time = self.get_clock().now()
            end_time = self.get_clock().now()
            time_difference = end_time - start_time
            seconds_diff = time_difference.nanoseconds / 1e9
            
            while(seconds_diff < 2):     #activate for 2 seconds 
                # publish -> angular.z 90 degrees right
                self.twist_cmd.linear.x = 0.0
                self.twist_cmd.angular.z = 0.0
                # Publish the message.
                self.twist_publisher.publish(self.twist_cmd)
                # Log the published message for verification.
                #self.get_logger().info(f'Publishing: Linear.x="{self.twist_cmd.linear.x:.2f}" Angular.z="{self.twist_cmd.angular.z:.2f}"')
            
                end_time = self.get_clock().now()
                time_difference = end_time - start_time
                seconds_diff = time_difference.nanoseconds / 1e9
                
            self.state = 'STOPPING'
        
        elif self.state == 'MOVE_FORWARD':
            start_time = self.get_clock().now()
            end_time = self.get_clock().now()
            time_difference = end_time - start_time
            seconds_diff = time_difference.nanoseconds / 1e9
            
            while(seconds_diff < 2):     #activate for 2 seconds 
                # publish -> angular.z 90 degrees right
                self.twist_cmd.linear.x = 0.2
                self.twist_cmd.angular.z = 0.01
                # Publish the message.
                self.twist_publisher.publish(self.twist_cmd)
                # Log the published message for verification.
                #self.get_logger().info(f'Publishing: Linear.x="{self.twist_cmd.linear.x:.2f}" Angular.z="{self.twist_cmd.angular.z:.2f}"')
            
                end_time = self.get_clock().now()
                time_difference = end_time - start_time
                seconds_diff = time_difference.nanoseconds / 1e9
                
            self.state = 'SEARCHING'
        
        elif self.state == 'U_TURN_LEFT':
            start_time = self.get_clock().now()
            end_time = self.get_clock().now()
            time_difference = end_time - start_time
            seconds_diff = time_difference.nanoseconds / 1e9
            
            while(seconds_diff < 2):     #activate for 2 seconds (offset to the right for smooth turn)
                # publish -> angular.z 90 degrees right
                self.twist_cmd.linear.x = 0.2
                self.twist_cmd.angular.z = 0.6
                # Publish the message.
                self.twist_publisher.publish(self.twist_cmd)
                # Log the published message for verification.
                #self.get_logger().info(f'Publishing: Linear.x="{self.twist_cmd.linear.x:.2f}" Angular.z="{self.twist_cmd.angular.z:.2f}"')
            
                end_time = self.get_clock().now()
                time_difference = end_time - start_time
                seconds_diff = time_difference.nanoseconds / 1e9

            start_time = self.get_clock().now()
            seconds_diff = 0
            while(seconds_diff < 7):     #activate for 6 seconds (make the big turn left)
                # publish -> angular.z 90 degrees right
                self.twist_cmd.linear.x = 0.2
                self.twist_cmd.angular.z = -0.7
                # Publish the message.
                self.twist_publisher.publish(self.twist_cmd)
                # Log the published message for verification.
                #self.get_logger().info(f'Publishing: Linear.x="{self.twist_cmd.linear.x:.2f}" Angular.z="{self.twist_cmd.angular.z:.2f}"')
            
                end_time = self.get_clock().now()
                time_difference = end_time - start_time
                seconds_diff = time_difference.nanoseconds / 1e9
                
            start_time = self.get_clock().now()
            seconds_diff = 0
            while(seconds_diff < 1):     #activate for 6 seconds (turn right to straighten the robot)
                # publish -> angular.z 90 degrees right
                self.twist_cmd.linear.x = 0.2
                self.twist_cmd.angular.z = 0.4
                # Publish the message.
                self.twist_publisher.publish(self.twist_cmd)
                # Log the published message for verification.
                #self.get_logger().info(f'Publishing: Linear.x="{self.twist_cmd.linear.x:.2f}" Angular.z="{self.twist_cmd.angular.z:.2f}"')
            
                end_time = self.get_clock().now()
                time_difference = end_time - start_time
                seconds_diff = time_difference.nanoseconds / 1e9
    
            self.state = 'SEARCHING'
                        
        elif self.state == 'U_TURN_RIGHT':
            start_time = self.get_clock().now()
            end_time = self.get_clock().now()
            time_difference = end_time - start_time
            seconds_diff = time_difference.nanoseconds / 1e9     
                  
            while(seconds_diff < 2):     #activate for 2 seconds (offset to the right for smooth turn)
                # publish -> angular.z 90 degrees right
                self.twist_cmd.linear.x = 0.2
                self.twist_cmd.angular.z = -0.5
                # Publish the message.
                self.twist_publisher.publish(self.twist_cmd)
                # Log the published message for verification.
                #self.get_logger().info(f'Publishing: Linear.x="{self.twist_cmd.linear.x:.2f}" Angular.z="{self.twist_cmd.angular.z:.2f}"')
            
                end_time = self.get_clock().now()
                time_difference = end_time - start_time
                seconds_diff = time_difference.nanoseconds / 1e9

            start_time = self.get_clock().now()
            seconds_diff = 0
            while(seconds_diff < 7):     #activate for 6 seconds (make the big turn left)
                # publish -> angular.z 90 degrees right
                self.twist_cmd.linear.x = 0.2
                self.twist_cmd.angular.z = 0.8
                # Publish the message.
                self.twist_publisher.publish(self.twist_cmd)
                # Log the published message for verification.
                #self.get_logger().info(f'Publishing: Linear.x="{self.twist_cmd.linear.x:.2f}" Angular.z="{self.twist_cmd.angular.z:.2f}"')
            
                end_time = self.get_clock().now()
                time_difference = end_time - start_time
                seconds_diff = time_difference.nanoseconds / 1e9
        
            start_time = self.get_clock().now()
            seconds_diff = 0
            while(seconds_diff < 1):     #activate for 6 seconds (turn right to straighten the robot)
                # publish -> angular.z 90 degrees right
                self.twist_cmd.linear.x = 0.2
                self.twist_cmd.angular.z = -0.2
                # Publish the message.
                self.twist_publisher.publish(self.twist_cmd)
                # Log the published message for verification.
                #self.get_logger().info(f'Publishing: Linear.x="{self.twist_cmd.linear.x:.2f}" Angular.z="{self.twist_cmd.angular.z:.2f}"')
                end_time = self.get_clock().now()
                time_difference = end_time - start_time
                seconds_diff = time_difference.nanoseconds / 1e9
                
            self.state = 'SEARCHING'

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
