from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='four_wheel_bot',
            executable='robot_controller',
            name='robot_controller',
            output='screen',
        ),
        Node(
            package='four_wheel_bot',
            executable='camera_publisher',
            name='camera_publisher',
            output='screen',
        ),
        Node(
            package='four_wheel_bot',
            executable='servo_controller',
            name='servo_controller',
            output='screen',
        ),
        Node(
            package='four_wheel_bot',
            executable='ultrasonic_node',
            name='ultrasonic_node',
            output='screen',
        ),
        Node(
            package='four_wheel_bot',
            executable='adc_sensor_node',
            name='adc_sensor_node',
            output='screen',
        ),
        Node(
            package='four_wheel_bot',
            executable='led_controller_node',
            name='led_controller_node',
            output='screen',
        ),
        Node(
            package='four_wheel_bot',
            executable='buzzer_controller_node',
            name='buzzer_controller_node',
            output='screen',
        ),
        Node(
            package='four_wheel_bot',
            executable='line_tracking_node',
            name='line_tracking_node',
            output='screen',
        ),
        Node(
            package='four_wheel_bot',
            executable='oled_display_node',
            name='oled_display_node',
            output='screen',
        ),
    ])

