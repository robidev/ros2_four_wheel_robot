import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import RPi.GPIO as GPIO

class Buzzer:
    BUZZER_PIN = 17

    def __init__(self):
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.BUZZER_PIN, GPIO.OUT)
        GPIO.output(self.BUZZER_PIN, False)

    def set(self, on: bool):
        GPIO.output(self.BUZZER_PIN, on)

class BuzzerControllerNode(Node):
    def __init__(self):
        super().__init__('buzzer_controller_node')
        self.buzzer = Buzzer()
        self.subscription = self.create_subscription(
            Bool,
            '/buzzer',
            self.buzzer_callback,
            10
        )
        self.get_logger().info("Buzzer Controller Node started.")

    def buzzer_callback(self, msg: Bool):
        state = msg.data
        self.buzzer.set(state)
        self.get_logger().info(f"Buzzer {'ON' if state else 'OFF'}")

    def destroy_node(self):
        self.buzzer.set(False)
        GPIO.cleanup(self.buzzer.BUZZER_PIN)
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = BuzzerControllerNode()
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

# Run the node:
# 
# ros2 run four_wheel_bot buzzer_controller_node
# 
# Turn the buzzer on for 3 seconds:
# 
# ros2 topic pub /buzzer std_msgs/msg/Bool "data: true"
# sleep 3
# ros2 topic pub /buzzer std_msgs/msg/Bool "data: false"
