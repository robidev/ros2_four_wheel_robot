import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from four_wheel_bot.Servo import Servo

class ServoControllerNode(Node):
    def __init__(self):
        super().__init__('servo_controller')
        self.servo = Servo()
        self._subscriptions = []

        # Subscribe to multiple servo channels
        for ch in [0, 1]:
            topic = f'servo/s{ch}/angle'
            sub = self.create_subscription(
                Float32,
                topic,
                lambda msg, ch=ch: self.set_servo_angle(ch+8, msg.data),
                10
            )
            self._subscriptions.append(sub)
            self.get_logger().info(f'Subscribed to {topic}')

    def set_servo_angle(self, channel, angle):
        angle = max(0.0, min(180.0, angle))  # clamp between 0–180
        self.get_logger().info(f'Setting servo {channel} to {angle:.1f}°')
        self.servo.set_angle(channel, angle)

def main(args=None):
    rclpy.init(args=args)
    node = ServoControllerNode()
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
