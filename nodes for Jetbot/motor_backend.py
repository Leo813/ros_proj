
from Adafruit_MotorHAT import Adafruit_MotorHAT
import time

class Motor():
    def __init__(self, driver, channel):

        self._driver = driver
        self._motor = self._driver.getMotor(channel)
        if (channel == 1):
            self._ina = 1
            self._inb = 0
        else:
            self._ina = 2
            self._inb = 3


    def setspeed(self, value):
        """Sets motor value between [-1, 1]"""
        mapped_value = int(255.0 * value)
        speed = min(max(abs(mapped_value), 0), 255)
        self._motor.setSpeed(speed)
        if mapped_value < 0:
            self._motor.run(Adafruit_MotorHAT.FORWARD)
            # The two lines below are required for the Waveshare JetBot Board only
            self._driver._pwm.setPWM(self._ina, 0, 0)
            self._driver._pwm.setPWM(self._inb, 0, speed * 16)
        else:
            self._motor.run(Adafruit_MotorHAT.BACKWARD)
            # The two lines below are required for the Waveshare JetBot Board only
            self._driver._pwm.setPWM(self._ina, 0, speed * 16)
            self._driver._pwm.setPWM(self._inb, 0, 0)

    def _release(self):
        """Stops motor by releasing control"""
        self._motor.run(Adafruit_MotorHAT.RELEASE)
        # The two lines below are required for the Waveshare JetBot Board only
        self._driver._pwm.setPWM(self._ina, 0, 0)
        self._driver._pwm.setPWM(self._inb, 0, 0)


class Robot():

    i2c_bus = 1
    left_motor_channel = 1
    right_motor_channel = 2

    def __init__(self):

        self.motor_driver = Adafruit_MotorHAT(i2c_bus=self.i2c_bus)
        self.left_motor = Motor(self.motor_driver, channel=self.left_motor_channel)
        self.right_motor = Motor(self.motor_driver, channel=self.right_motor_channel)

    def set_motors(self, left_speed, right_speed):
        self.left_motor.setspeed(left_speed)
        self.right_motor.setspeed(right_speed)

    def forward(self, speed=1.0):
        self.left_motor.setspeed(speed)
        self.right_motor.setspeed(speed)

    def backward(self, speed=1.0):
        self.left_motor.setspeed(-speed)
        self.right_motor.setspeed(-speed)

    def left(self, speed=1.0):
        self.left_motor.setspeed(-speed)
        self.right_motor.setspeed(speed)

    def right(self, speed=1.0):

        self.left_motor.setspeed(speed)
        self.right_motor.setspeed(-speed)

    def stop(self):
        self.left_motor.setspeed(0)
        self.right_motor.setspeed(0)



if __name__ == '__main__':
    robot = Robot()

    robot.right(0.3)
    # robot.set_motors(100,100)

    time.sleep(0.7)
    robot.stop()