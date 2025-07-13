import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from four_wheel_bot.Adc import Adc

class AdcSensorNode(Node):
    def __init__(self):
        super().__init__('adc_sensor_node')
        self.adc = Adc()

        self.ldr_left_pub = self.create_publisher(Float32, '/sensors/ldr_left', 10)
        self.ldr_right_pub = self.create_publisher(Float32, '/sensors/ldr_right', 10)
        self.battery_pub = self.create_publisher(Float32, '/sensors/battery_voltage', 10)

        self.timer = self.create_timer(1.0, self.publish_adc_values)  # every 1s

        self.get_logger().info("ADC Sensor Node started")

    def publish_adc_values(self):
        try:
            left_ldr = self.adc.recvADC(0)
            right_ldr = self.adc.recvADC(1)
            battery_voltage = self.adc.recvADC(2) * 5  # scale if resistor divider used

            self.ldr_left_pub.publish(Float32(data=left_ldr))
            self.ldr_right_pub.publish(Float32(data=right_ldr))
            self.battery_pub.publish(Float32(data=battery_voltage))

            self.get_logger().debug(
                f"LDR L: {left_ldr:.2f} V | R: {right_ldr:.2f} V | Battery: {battery_voltage:.2f} V"
            )
        except Exception as e:
            self.get_logger().warn(f"ADC reading failed: {e}")

    def destroy_node(self):
        self.adc.i2cClose()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = AdcSensorNode()
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
