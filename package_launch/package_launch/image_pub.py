#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class WebcamPublisher(Node):
    def __init__(self):
        super().__init__('webcam_publisher')
        self.publisher_ = self.create_publisher(Image, '/camera', 10)
        self.timer = self.create_timer(0.01, self.timer_callback)  # 10 Hz
        self.cap = cv2.VideoCapture(0)  # 0 is default webcam
        self.bridge = CvBridge()
        if not self.cap.isOpened():
            self.get_logger().error("Could not open webcam!")

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("Failed to capture image")
            return
        
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        msg = self.bridge.cv2_to_imgmsg(gray_frame, encoding="mono8")
        self.publisher_.publish(msg)
        self.get_logger().info("Published webcam image")

def main(args=None):
    rclpy.init(args=args)
    node = WebcamPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.cap.release()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
