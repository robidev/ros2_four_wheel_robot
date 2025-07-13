import time
#from Motor import *
import RPi.GPIO as GPIO
#from servo import *
from four_wheel_bot.PCA9685 import PCA9685
import random

class Ultrasonic:
    def __init__(self):
        GPIO.setwarnings(False)
        self.trigger_pin = 27
        self.echo_pin = 22
        self.MAX_DISTANCE = 300  # define the maximum measuring distance, unit: cm
        self.timeOut = self.MAX_DISTANCE * 60  # calculate timeout according to the maximum measuring distance
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.trigger_pin, GPIO.OUT)
        GPIO.setup(self.echo_pin, GPIO.IN)

    def pulseIn(self, pin, level, timeOut):  # obtain pulse time of a pin under timeOut
        t0 = time.time()
        while (GPIO.input(pin) != level):
            if ((time.time() - t0) > timeOut * 0.000001):
                return 0;
        t0 = time.time()
        while (GPIO.input(pin) == level):
            if ((time.time() - t0) > timeOut * 0.000001):
                return 0;
        pulseTime = (time.time() - t0) * 1000000
        return pulseTime

    def get_distance(self):  # get the measurement results of ultrasonic module,with unit: cm
        distance_cm = [0, 0, 0, 0, 0]
        for i in range(5):
            GPIO.output(self.trigger_pin, GPIO.HIGH)  # make trigger_pin output 10us HIGH level
            time.sleep(0.00001)  # 10us
            GPIO.output(self.trigger_pin, GPIO.LOW)  # make trigger_pin output LOW level
            pingTime = self.pulseIn(self.echo_pin, GPIO.HIGH, self.timeOut)  # read plus time of echo_pin
            distance_cm[i] = pingTime * 340.0 / 2.0 / 10000.0  # calculate distance with sound speed 340m/s
        distance_cm = sorted(distance_cm)
        final_distance = distance_cm[2]  # 取排序后的中间值

        # 如果测量值等于0，转换为255
        if final_distance == 0:
            final_distance = 255

        return int(final_distance)  # 返回处理后的整数距离值
        #return int(distance_cm[2])
    
#    def run(self):
#        self.PWM = Motor()
#        self.pwm_S = Servo()
#        while True:
#            M = self.get_distance()
#            if M <= 20:
#                self.PWM.setMotorModel(-1000, -1000, -1000, -1000)
#                time.sleep(0.3)  
#                if 50 >= random.randint(1, 100):
#                    self.PWM.setMotorModel(-2000, -2000, 2000, 2000)
#                else:
#                    self.PWM.setMotorModel(2000, 2000, -2000, -2000)
#                time.sleep(0.3)  
#
#            elif 20 < M <= 30:
#                self.PWM.setMotorModel(0, 0, 0, 0)
#                time.sleep(0.2)
#                if 50 >= random.randint(1, 100):
#                    self.PWM.setMotorModel(-2000, -2000, 2000, 2000)
#                else:
#                    self.PWM.setMotorModel(2000, 2000, -2000, -2000)
#                time.sleep(0.3)
#            else:  
#                self.PWM.setMotorModel(1000, 1000, 1000, 1000)


#ultrasonic = Ultrasonic()
# Main program logic follows:
#if __name__ == '__main__':
#    print('Program is starting ... ')
#    try:
#        ultrasonic.run()
#    except KeyboardInterrupt:  # When 'Ctrl+C' is pressed, the child program destroy() will be  executed.
#        PWM.setMotorModel(0, 0, 0, 0)
#        ultrasonic.pwm_S.setServoPwm('0', 90)
