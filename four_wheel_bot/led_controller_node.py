import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from four_wheel_bot.Led import Led

class LedControllerNode(Node):
    def __init__(self):
        super().__init__('led_controller_node')
        self.led = Led()

        if self.led.Ledsupported:
            self.subscriber = self.create_subscription(
                Int32MultiArray,
                '/led/set_color',
                self.set_led_color_callback,
                10
            )
            self.get_logger().info("LED Controller Node started. Listening on /led/set_color")
        else:
            self.get_logger().warn("LED hardware not supported or SPI not enabled.")

    def set_led_color_callback(self, msg: Int32MultiArray):
        if len(msg.data) != 4:
            self.get_logger().warn("Expected [index, R, G, B], got: " + str(msg.data))
            return

        index, r, g, b = msg.data
        self.get_logger().info(f"Setting LED {index} to RGB({r}, {g}, {b})")
        self.led.ledIndex(index, r, g, b)

    def destroy_node(self):
        if self.led.Ledsupported:
            self.led.strip.set_all_led_color(0, 0, 0)
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = LedControllerNode()
    try:
        #rclpy.spin(node)
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.3)  # 300ms wait
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
# ros2 topic pub /led/set_color std_msgs/msg/Int32MultiArray "{data: [0, 255, 0, 0]}"
