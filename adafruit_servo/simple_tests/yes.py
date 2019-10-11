# Test nodding head "Yes"

from board import SCL, SDA
import busio
from adafruit_servokit import ServoKit
import adafruit_motor.servo
import time

# Import the PCA9685 module.
from adafruit_pca9685 import PCA9685

# Create the I2C bus interface.
i2c_bus = busio.I2C(SCL, SDA)

# Create a simple PCA9685 class instance.
pca = PCA9685(i2c_bus)

# Set the PWM frequency to 50hz.
pca.frequency = 50

# Initialize ServoKit for 16-channel control
kit = ServoKit(channels=16)

# Start up Servo class
servo = adafruit_motor.servo.Servo(pca)

# Configure for commodity metal gear micro servos
kit.servo[0].actuation_range=180
kit.servo[0].set_pulse_width_range(500,2500)
kit.servo[1].actuation_range=180
kit.servo[1].set_pulse_width_range(500,2500)
kit.servo[2].actuation_range=180
kit.servo[2].set_pulse_width_range(500,2500)

# Nodding up-down, two cycles.
kit.servo[0].angle=75
kit.servo[1].angle=105
kit.servo[2].angle=90
time.sleep(0.3)
kit.servo[0].angle=105
kit.servo[1].angle=75
kit.servo[2].angle=90
time.sleep(0.3)
kit.servo[0].angle=75
kit.servo[1].angle=105
kit.servo[2].angle=90
time.sleep(0.3)
kit.servo[0].angle=105
kit.servo[1].angle=75
kit.servo[2].angle=90
time.sleep(0.3)

# Reset to neutral position
kit.servo[0].angle=90
kit.servo[1].angle=90
kit.servo[2].angle=90
