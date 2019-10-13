"""
Control an Arduino I2C 16-channel PWM via ROS topics.

ROS Kinetic is tied to Python 2
https://www.ros.org/reps/rep-0003.html#kinetic-kame-may-2016-may-2021

Arduino CircuitPython library is tied to Python 3
https://learn.adafruit.com/circuitpython-on-raspberrypi-linux/installing-circuitpython-on-raspberry-pi

ROS bridge suite is used to bridge these worlds via WebSocket.
http://wiki.ros.org/rosbridge_suite

And the 'roslibpy' library is used here to make the Python code easier.
https://roslibpy.readthedocs.io
"""
# Standard Python libraries
import time

# Python library for connecting to ROS via ROS bridge suite WebSocket server
import roslibpy

# IP address or name of machine running ROS master
ROS_MASTER = "192.168.0.111"

# WebSocket port listened by Rosbridge WebSocket on ROS master
ROS_BRIDGE_PORT = 9090

# We may be up and running before ROS master is. Wait up to this many seconds
# for ROS master node to respond.
ROS_CONNECT_TIMEOUT = 120

def progress(text):
  """
  Progress report as code executes. Normally print() to console output but can
  be redirected.
  """
  print(text)

class ServoNeckGamepad:
  """
  Listens to gamepad input published to '/joy' topic and command neck servos to
  react accordingly
  """
  def __init__(self):
    """
    Class constructor sets up an instance of roslibpy to use
    """
    progress("Creating client for {0}:{1}".format(ROS_MASTER, ROS_BRIDGE_PORT))
    self.ros_client = roslibpy.Ros(host=ROS_MASTER, port=ROS_BRIDGE_PORT)

  def run(self):
    """
    Main entry point of ServoNeckGamepad class
    """
    try:
      progress("Connecting to ROS master")
      # self.ros_client.run_forever makes more sense, but it doesn't have a
      # timeout parameter
      self.ros_client.run(ROS_CONNECT_TIMEOUT)

      # When we're connected and ready, get to work.
      self.ros_client.on_ready(self.on_ready_callback)

      # Since we can't use run_forever(), we end up with run(TIMEOUT)
      # and check connected status every 5 seconds
      while self.ros_client.is_connected:
        time.sleep(5.0)
    finally:
      progress("Terminating ROS client")
      self.ros_client.terminate()

  def on_ready_callback(self):
    """
    When ROS connection is ready, subscribe to /joy topic
    """
    topicJoy = roslibpy.Topic(self.ros_client, '/joy', 'sensor_msgs/Joy')
    topicJoy.subscribe(self.joystick_callback)

  def joystick_callback(self, message):
    """
    When a /joy topic message arrives, examine buttons state given in
    message['buttons'] to determine action
    Reference: http://docs.ros.org/kinetic/api/sensor_msgs/html/msg/Joy.html
    """
    progress("/joy buttons: {1} @ {0}".format(time.time(),message['buttons']))

if __name__ == "__main__":
  sng = ServoNeckGamepad()
  sng.run()