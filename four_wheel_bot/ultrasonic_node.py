import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from four_wheel_bot.Ultrasonic import Ultrasonic

class UltrasonicNode(Node):
    def __init__(self):
        super().__init__('ultrasonic_node')
        self.sensor = Ultrasonic()
        self.publisher = self.create_publisher(Range, 'ultrasonic/distance', 10)
        self.timer = self.create_timer(0.2, self.publish_distance)  # 5Hz

        self.get_logger().info('Ultrasonic sensor node started')

    def publish_distance(self):
        distance = self.sensor.get_distance() / 100.0  # convert cm to meters
        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "ultrasonic_sensor"
        msg.radiation_type = Range.ULTRASOUND
        msg.field_of_view = 0.3  # radians; adjust if needed
        msg.min_range = 0.02
        msg.max_range = 3.0
        msg.range = float(distance)

        self.publisher.publish(msg)
        self.get_logger().debug(f"Distance: {msg.range:.2f} m")

def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicNode()
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
