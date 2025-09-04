import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from roboflowoak import RoboflowOak
import cv2

class PeopleDetectorNode(Node):
    def __init__(self):
        super().__init__('people_detector_node')

        # Publisher for detection summary
        self.pub = self.create_publisher(Twist, 'cmd_val', 10)
        self.twist_cmd = Twist()

        # Init Roboflow OAK model (Roboflow people detection model ID/version)
        self.rf = RoboflowOak(
            model="people-detection-general",    # People detection model
            confidence=0.4,                   # filter weak detections
            overlap=0.5,
            version="5",                      
            api_key="qS4FEZsIHOVMnr0XfFoP",
            rgb=True,
            depth=True,
            device=None,
            blocking=True
        )

        self.get_logger().info(" People Detector Node started")

        # Timer 
        self.timer = self.create_timer(0.1, self.detect_callback)  # 10 Hz

    def detect_callback(self):
        #frame - frame after preprocs, with predictions
        #raw_frame - original frame from  OAK
        #depth - depth map for raw_frame, center-rectified to the center camera
        
        result, frame, raw_frame, depth = self.rf.detect()
        predictions = result["predictions"]

        # Filter only people
        people = [p for p in predictions if p.class_name == "person"]
        
        while len(people) > 0:
            
            #  publish to the vesc that it should stop moving the robot
            self.twist_cmd.angular.z = 0.0
            self.twist_cmd.linear.x = 0.0
            self.pub.publish(self.twist_cmd)


       
def main():
    rclpy.init()
    node = PeopleDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
