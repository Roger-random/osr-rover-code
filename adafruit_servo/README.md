# Adafruit Servo Control

Software integration of Adafruit's servo library in order to control servos on
board a rover. Originally written for their PCA9685-based I2C 16-channel
[product id 815](https://www.adafruit.com/product/815), the use of Adafruit's
[CircuitPython servo library](https://learn.adafruit.com/16-channel-pwm-servo-driver/python-circuitpython)
should make it adaptable to other Adafruit-supported servo control hardware.

## ROS communication:

ROS Kinetic is tied to Python 2, but Adafruit's CircuitPython libraries are
Python 3 meaning it could not directly use the ROS `rospy` library for
ROS messages. Instead it will indirectly communicate using JSON over
WebSockets by using [rosbridge_suite](wiki.ros.org/rosbridge_suite) and
[roslibpy](https://roslibpy.readthedocs.io/en/latest/) libraries.

This decoupling also permits servo logic to be moved off of the main Raspberry
Pi brain of the rover. Including hardware that could not otherwise run ROS.
