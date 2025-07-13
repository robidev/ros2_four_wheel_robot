import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from four_wheel_bot.Motor import Motor

class RobotControllerNode(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self.motor = Motor()
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        self.get_logger().info('Robot Controller Node Started')

    def cmd_vel_callback(self, msg: Twist):
        linear = msg.linear.x  # forward/backward
        angular = msg.angular.z  # rotation

        # Convert velocities to motor PWM duty cycle
        base_speed = 2000  # adjust this as needed
        turn_speed = 1000  # angular adjustment

        left_speed = base_speed * linear - turn_speed * angular
        right_speed = base_speed * linear + turn_speed * angular

        # Assign to each wheel (simplified)
        duty1 = int(left_speed)   # left upper
        duty2 = int(left_speed)   # left lower
        duty3 = int(right_speed)  # right upper
        duty4 = int(right_speed)  # right lower
        
        self.get_logger().info(f'Setting wheels to: LU={duty1}, LL={duty2}, RU={duty3}, RL={duty4}')
        self.motor.setMotorModel(duty1, duty2, duty3, duty4)

def main(args=None):
    rclpy.init(args=args)
    node = RobotControllerNode()
    try:
        #rclpy.spin(node)
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)  # 100ms wait
    except KeyboardInterrupt:
        pass
    finally:
        node.motor.setMotorModel(0, 0, 0, 0)  # stop the robot
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
