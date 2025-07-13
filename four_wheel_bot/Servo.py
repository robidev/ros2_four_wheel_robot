from four_wheel_bot.PCA9685 import PCA9685

class Servo:
    def __init__(self):
        self.pwm = PCA9685(0x40, debug=True)
        self.pwm.setPWMFreq(50)  # 50Hz for servo
        self.set_angle(0, 90)
        self.set_angle(1, 90)

    def set_angle(self, channel: int, angle: float):
        # Convert angle (0–180) to pulse width (typically 500–2500us)
        pulse_min = 500
        pulse_max = 2500
        pulse = pulse_min + (pulse_max - pulse_min) * angle / 180.0
        self.pwm.setServoPulse(channel, pulse)
