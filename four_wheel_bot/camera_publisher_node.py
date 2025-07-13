import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from picamera2 import Picamera2
import cv2

class CameraPublisherNode(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        self.publisher_ = self.create_publisher(Image, 'image_raw', 10)
        self.bridge = CvBridge()

        # Initialize Picamera2
        self.picam2 = Picamera2()
        config = self.picam2.create_video_configuration(main={"size": (640, 480)})
        self.picam2.configure(config)
        self.picam2.start()

        self.timer = self.create_timer(0.1, self.timer_callback)  # 10Hz
        self.get_logger().info('Camera Publisher Node Started')

    def timer_callback(self):
        # Capture frame from Picamera2
        frame = self.picam2.capture_array()
        if frame is not None:
            # Convert 4-channel (BGRA) to 3-channel (BGR)
            if frame.shape[2] == 4:
                frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)

            # Convert BGR image to ROS2 Image message
            msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.publisher_.publish(msg)
        else:
            self.get_logger().warn("Empty frame captured")

def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisherNode()
    try:
        #rclpy.spin(node)
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)  # 100ms wait
    except KeyboardInterrupt:
        pass
    finally:
        node.picam2.stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
