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
    progress("Creating client for {0}:{1}".format(ROS_MASTER, ROS_BRIDGE_PORT))
    self.ros_client = roslibpy.Ros(host=ROS_MASTER, port=ROS_BRIDGE_PORT)

  def run(self):
    """
    Main entry point of ServoNeckGamepad class
    """
    progress("Connecting to ROS master")
    self.ros_client.run(ROS_CONNECT_TIMEOUT)
    progress("Connection status: {0}".format(self.ros_client.is_connected))
    self.ros_client.terminate()

if __name__ == "__main__":
  sng = ServoNeckGamepad()
  sng.run()