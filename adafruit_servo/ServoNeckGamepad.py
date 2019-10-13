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
import threading
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
    Class constructor
    """
    # ROS bridge
    progress("Creating client for {0}:{1}".format(ROS_MASTER, ROS_BRIDGE_PORT))
    self.ros_client = roslibpy.Ros(host=ROS_MASTER, port=ROS_BRIDGE_PORT)

    # Control access to Adafruit servo library to ensure only one thread is
    # communicatng over I2C at any given time.
    self.mutex = threading.Lock()

    # Flags to control path of execution
    self.wait_for_release = False
    self.motion_complete = True

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
    topicJoy = roslibpy.Topic(self.ros_client, '/joy', 'sensor_msgs/Joy', throttle_rate=50, queue_length=1)
    topicJoy.subscribe(self.joystick_callback)

  def joystick_callback(self, message):
    """
    When a /joy topic message arrives, examine buttons state given in
    message['buttons'] to determine action
    Reference: http://wiki.ros.org/joy
               http://docs.ros.org/kinetic/api/sensor_msgs/html/msg/Joy.html
    """
    if self.motion_complete:
      A,B,X,Y = message['buttons'][0:4]
      if self.wait_for_release:
        if [A,B,X,Y] == [0,0,0,0]:
          self.wait_for_release = False
      else:
        if A == 1:
          self.motion_start()
          self.ros_client.call_in_thread(self.motion_shake)
        elif X == 1:
          self.motion_start()
          self.ros_client.call_in_thread(self.motion_huh)
        elif Y == 1:
          self.motion_start()
          self.ros_client.call_in_thread(self.motion_nod)

  def motion_start(self):
    """
    Call this before launching another thread for actual motion
    """
    self.motion_complete = False
    self.wait_for_release = True

  def motion_end(self):
    """
    Call this from motion method to signal completion
    """
    self.motion_complete = True

  def motion_nod(self):
    """
    Nod head "yes"
    """
    with self.mutex:
      print("Nodding start")
      time.sleep(3)
      print("Nodding done")
      self.motion_end()

  def motion_shake(self):
    """
    Shake head "no"
    """
    with self.mutex:
      print("Shaking")
      time.sleep(3)
      print("Shaking done")
      self.motion_end()

  def motion_huh(self):
    """
    Tilt head puzzled "huh"
    """
    with self.mutex:
      print("Puzzled")
      time.sleep(3)
      print("Puzzled done")
      self.motion_end()

if __name__ == "__main__":
  sng = ServoNeckGamepad()
  sng.run()