import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8MultiArray
import RPi.GPIO as GPIO

class LineTrackingNode(Node):
    def __init__(self):
        super().__init__('line_tracking_node')

        # GPIO pin setup
        self.left_pin = 14
        self.center_pin = 15
        self.right_pin = 23

        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.left_pin, GPIO.IN)
        GPIO.setup(self.center_pin, GPIO.IN)
        GPIO.setup(self.right_pin, GPIO.IN)

        # Publisher
        self.publisher = self.create_publisher(Int8MultiArray, '/line_tracking', 10)
        self.timer = self.create_timer(0.05, self.read_sensors)  # 20 Hz

        self.get_logger().info('Line tracking node started')

    def read_sensors(self):
        left = GPIO.input(self.left_pin)
        center = GPIO.input(self.center_pin)
        right = GPIO.input(self.right_pin)

        msg = Int8MultiArray()
        msg.data = [left, center, right]
        self.publisher.publish(msg)

    def destroy_node(self):
        GPIO.cleanup([self.left_pin, self.center_pin, self.right_pin])
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = LineTrackingNode()
    try:
        #rclpy.spin(node)
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)  # 100ms wait
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

# Test
# 
# ros2 run four_wheel_bot line_tracking_node
# 
# And monitor:
# 
# ros2 topic echo /line_tracking
# 
# Expected output:
# data: [1, 0, 1] — for example, if only center sensor is off the line.
