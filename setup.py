from setuptools import setup
import os
from glob import glob
from setuptools import setup, find_packages


package_name = 'four_wheel_bot'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(), #[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/ament_index/resource_index/packages',['resource/' + package_name]),
        ],
    install_requires=[
        'setuptools',
        'rclpy',
        'numpy',
        'opencv-python',
    ],
    zip_safe=True,
    maintainer='Robidev',
    maintainer_email='robin.dev@gmail.com',
    description='ROS 2 nodes for controlling a 4-wheel Raspberry Pi robot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_controller = four_wheel_bot.robot_controller_node:main',
            'camera_publisher = four_wheel_bot.camera_publisher_node:main',
            'servo_controller = four_wheel_bot.servo_controller_node:main',
            'ultrasonic_node = four_wheel_bot.ultrasonic_node:main',
            'adc_sensor_node = four_wheel_bot.adc_sensor_node:main',
            'led_controller_node = four_wheel_bot.led_controller_node:main',
            'buzzer_controller_node = four_wheel_bot.buzzer_controller_node:main',
            'line_tracking_node = four_wheel_bot.line_tracking_node:main',
            'oled_display_node = four_wheel_bot.oled_display_node:main',
        ],
    },
)
